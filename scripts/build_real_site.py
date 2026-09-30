#!/usr/bin/env python3
"""Build a Gazebo world of a real site from official open geodata.

Inputs (NRW OpenGeodata, 1 km tiles, EPSG:25832 + DHHN2016 heights):
    dgm1_32_<E>_<N>_1_nw_*.tif        1 m digital terrain model (bare ground)
    dop10rgbi_32_<E>_<N>_1_nw_*.jp2   10 cm aerial orthophoto (R,G,B,IR)
    LoD2_32_<E>_<N>_1_NW.gml          LoD2 CityGML buildings (real roof shapes)
    3dm_32_<E>_<N>_1_nw.laz           airborne laser point cloud

Outputs (in the site's out_dir):
    <name>.sdf         Gazebo Harmonic world, PX4-ready (all required systems)
    meshes/*.obj       terrain (orthophoto-textured), buildings, trees, perimeter
    prior_map.laz      the real laser scan in world coordinates, plus the
                       perimeter you designed - i.e. "everything the operator
                       already knows". Intruders are NOT in it, which is what
                       change detection looks for.
    preview.png        orthophoto with footprints, trees, fence, actor paths

Why meshes and not <heightmap>: a mesh gives the orthophoto an exact 1:1 UV
mapping, and roofs can reuse the same texture. The drone's LiDAR sees
rendered geometry, so real building shapes are what make the sim "real";
appearance is secondary.

Coordinates: the site file uses metres relative to the bbox centre
(x = East, y = North). The world origin is placed at `launch_pad`, where PX4
spawns the vehicle, and heights are relative to the ground there.

With --export-scene it also writes scene/ - the engine-neutral package
(glTF terrain tiles + buildings, NDVI vegetation mask, scene_manifest.json)
that Unity loads now and Isaac Sim will load later. See scene_package.py.

Usage:
    ~/.venvs/geo/bin/python scripts/build_real_site.py sites/bonn_poppelsdorf.yaml
    ~/.venvs/geo/bin/python scripts/build_real_site.py sites/bonn_poppelsdorf.yaml --preview-only
    ~/.venvs/geo/bin/python scripts/build_real_site.py sites/bonn_poppelsdorf.yaml --export-scene
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import re
import sys
from dataclasses import dataclass
from pathlib import Path

import laspy
import mapbox_earcut as earcut
import numpy as np
import tifffile
import yaml
from lxml import etree
from PIL import Image, ImageDraw
from pyproj import Transformer
from scipy import ndimage
from shapely import contains_xy
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

from scene_package import facade_category

Image.MAX_IMAGE_PIXELS = None

NS = {
    "bldg": "http://www.opengis.net/citygml/building/1.0",
    "gml": "http://www.opengis.net/gml",
}

# Systems PX4 needs. A world that declares plugins inline does not get PX4's
# server.config applied, so every one of them must be listed here - missing
# magnetometer/air-pressure makes PX4 refuse to arm (no compass, no baro).
PX4_SYSTEMS = [
    ("gz-sim-physics-system", "gz::sim::systems::Physics"),
    ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
    ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
    ("gz-sim-contact-system", "gz::sim::systems::Contact"),
    ("gz-sim-imu-system", "gz::sim::systems::Imu"),
    ("gz-sim-air-pressure-system", "gz::sim::systems::AirPressure"),
    ("gz-sim-air-speed-system", "gz::sim::systems::AirSpeed"),
    ("gz-sim-apply-link-wrench-system", "gz::sim::systems::ApplyLinkWrench"),
    ("gz-sim-navsat-system", "gz::sim::systems::NavSat"),
    ("gz-sim-magnetometer-system", "gz::sim::systems::Magnetometer"),
]


# --------------------------------------------------------------------- frame --

@dataclass
class Frame:
    """Maps real-world coordinates (UTM32, DHHN2016) to world metres."""
    bbox_e0: float      # bbox lower-left easting
    bbox_n0: float      # bbox lower-left northing
    size: float         # bbox edge length (m)
    origin_e: float     # world origin (launch pad) easting
    origin_n: float
    origin_h: float     # ground height at the origin (DHHN2016 m)

    @property
    def centre(self) -> tuple[float, float]:
        return self.bbox_e0 + self.size / 2, self.bbox_n0 + self.size / 2

    def site_to_en(self, x: float, y: float) -> tuple[float, float]:
        """Site-file coordinates (relative to bbox centre) -> UTM."""
        ce, cn = self.centre
        return ce + x, cn + y

    def en_to_world(self, e, n):
        return np.asarray(e) - self.origin_e, np.asarray(n) - self.origin_n

    def site_to_world(self, x: float, y: float) -> tuple[float, float]:
        e, n = self.site_to_en(x, y)
        return e - self.origin_e, n - self.origin_n

    def uv(self, e, n):
        """Orthophoto texture coordinates for UTM points (OBJ: v=0 at bottom)."""
        return ((np.asarray(e) - self.bbox_e0) / self.size,
                (np.asarray(n) - self.bbox_n0) / self.size)


def tiles_for_bbox(e0: float, n0: float, size: float) -> list[tuple[int, int]]:
    return [(te, tn)
            for te in range(int(e0 // 1000), int((e0 + size - 1e-6) // 1000) + 1)
            for tn in range(int(n0 // 1000), int((n0 + size - 1e-6) // 1000) + 1)]


def find_tile(raw: Path, pattern: str) -> Path:
    hits = sorted(glob.glob(str(raw / pattern)))
    if not hits:
        sys.exit(f"missing input: {raw / pattern}")
    return Path(hits[-1])  # newest year if several


# ------------------------------------------------------------------- terrain --

class Terrain:
    """1 m ground-height grid covering the bbox, with bilinear sampling."""

    def __init__(self, raw: Path, e0: float, n0: float, size: float):
        self.e0, self.n0, self.size = e0, n0, size
        n = int(round(size))
        grid = np.full((n, n), np.nan, dtype=np.float32)  # row 0 = north edge
        for te, tn in tiles_for_bbox(e0, n0, size):
            path = find_tile(raw, f"dgm1_32_{te}_{tn}_1_nw_*.tif")
            with tifffile.TiffFile(path) as tif:
                page = tif.pages[0]
                data = page.asarray().astype(np.float32)
                sx, sy, _ = page.tags["ModelPixelScaleTag"].value
                _, _, _, ul_e, ul_n, _ = page.tags["ModelTiepointTag"].value
            assert sx == 1.0 and sy == 1.0, f"{path.name}: expected 1 m pixels"
            data[data < -1000] = np.nan  # nodata
            # overlap of this tile with the bbox, in bbox pixel indices
            c0 = int(round(ul_e - e0))
            r0 = int(round((n0 + size) - ul_n))
            rs, cs = max(r0, 0), max(c0, 0)
            re_, ce = min(r0 + data.shape[0], n), min(c0 + data.shape[1], n)
            if rs < re_ and cs < ce:
                grid[rs:re_, cs:ce] = data[rs - r0:re_ - r0, cs - c0:ce - c0]
        if np.isnan(grid).any():
            # small gaps: fill with nearest valid value
            idx = ndimage.distance_transform_edt(
                np.isnan(grid), return_distances=False, return_indices=True)
            grid = grid[tuple(idx)]
        self.grid = grid

    def height(self, e, n):
        """Bilinear ground height at UTM points (pixel centres at +0.5 m)."""
        e, n = np.asarray(e, float), np.asarray(n, float)
        scalar = e.ndim == 0
        col = np.atleast_1d(e - self.e0 - 0.5)
        row = np.atleast_1d((self.n0 + self.size) - n - 0.5)
        h = ndimage.map_coordinates(self.grid, [row, col], order=1, mode="nearest")
        return h[0] if scalar else h


# ---------------------------------------------------------------- orthophoto --

def build_orthophoto(raw: Path, e0: float, n0: float, size: float,
                     m_per_px: float, with_nir: bool = False):
    """Mosaic and crop the aerial photo to the bbox at the requested resolution.

    Returns the RGB image, or (RGB, near-infrared) when with_nir is set.
    """
    reduce = max(0, int(round(math.log2(m_per_px / 0.1))))
    px = 0.1 * 2 ** reduce
    out_n = int(round(size / px))
    canvas = Image.new("RGB", (out_n, out_n))
    nir_canvas = Image.new("L", (out_n, out_n)) if with_nir else None
    for te, tn in tiles_for_bbox(e0, n0, size):
        path = find_tile(raw, f"dop10rgbi_32_{te}_{tn}_1_nw_*.jp2")
        im = Image.open(path)
        im.reduce = reduce          # decode at lower resolution (JPEG2000 feature)
        im.load()
        bands = im.split()          # band 4 is near-infrared, not alpha
        rgb = Image.merge("RGB", bands[:3])
        # tile's upper-left, in bbox-canvas pixels (canvas row 0 = north edge)
        ox = int(round((te * 1000 - e0) / px))
        oy = int(round(((n0 + size) - (tn + 1) * 1000) / px))
        canvas.paste(rgb, (ox, oy))
        if with_nir:
            nir_canvas.paste(bands[3], (ox, oy))
        del im, bands, rgb
    return (canvas, nir_canvas) if with_nir else canvas


# ----------------------------------------------------------------- OBJ output --

class ObjWriter:
    """Minimal multi-material OBJ/MTL writer (explicit normals, optional UVs)."""

    def __init__(self):
        self.groups: list[tuple[str, np.ndarray, np.ndarray, np.ndarray | None,
                                np.ndarray]] = []

    def add(self, material: str, verts: np.ndarray, faces: np.ndarray,
            uvs: np.ndarray | None = None, normals: np.ndarray | None = None):
        if len(faces) == 0:
            return
        verts = np.asarray(verts, float)
        faces = np.asarray(faces, np.int64)
        if normals is None:  # per-vertex normals from face normals
            fn = np.cross(verts[faces[:, 1]] - verts[faces[:, 0]],
                          verts[faces[:, 2]] - verts[faces[:, 0]])
            normals = np.zeros_like(verts)
            for k in range(3):
                np.add.at(normals, faces[:, k], fn)
        ln = np.linalg.norm(normals, axis=1, keepdims=True)
        normals = normals / np.where(ln > 0, ln, 1)
        self.groups.append((material, verts, faces, uvs, normals))

    def triangle_count(self) -> int:
        return sum(len(g[2]) for g in self.groups)

    def write(self, obj_path: Path, materials: dict[str, dict]):
        mtl_path = obj_path.with_suffix(".mtl")
        with open(mtl_path, "w") as m:
            for name, spec in materials.items():
                m.write(f"newmtl {name}\n")
                kd = spec.get("kd", (0.8, 0.8, 0.8))
                m.write(f"Ka {kd[0]*0.6:.3f} {kd[1]*0.6:.3f} {kd[2]*0.6:.3f}\n")
                m.write(f"Kd {kd[0]:.3f} {kd[1]:.3f} {kd[2]:.3f}\n")
                m.write("Ks 0.05 0.05 0.05\nNs 10\nd 1.0\nillum 2\n")
                if "map_kd" in spec:
                    m.write(f"map_Kd {spec['map_kd']}\n")
                m.write("\n")
        vo = to = no = 0
        with open(obj_path, "w") as f:
            f.write(f"mtllib {mtl_path.name}\n")
            for i, (mat, v, fa, uv, nr) in enumerate(self.groups):
                f.write(f"o part{i}\n")
                f.write("".join(f"v {a:.3f} {b:.3f} {c:.3f}\n" for a, b, c in v))
                f.write("".join(f"vn {a:.4f} {b:.4f} {c:.4f}\n" for a, b, c in nr))
                if uv is not None:
                    f.write("".join(f"vt {a:.5f} {b:.5f}\n" for a, b in uv))
                f.write(f"usemtl {mat}\n")
                if uv is not None:
                    f.write("".join(
                        f"f {a+vo}/{a+to}/{a+no} {b+vo}/{b+to}/{b+no} {c+vo}/{c+to}/{c+no}\n"
                        for a, b, c in fa + 1))
                    to += len(uv)
                else:
                    f.write("".join(f"f {a+vo}//{a+no} {b+vo}//{b+no} {c+vo}//{c+no}\n"
                                    for a, b, c in fa + 1))
                vo += len(v)
                no += len(nr)


def grid_mesh(terrain: Terrain, frame: Frame, step: float):
    """Regular triangulated grid over the bbox, heights from the terrain model."""
    n = int(round(frame.size / step)) + 1
    es = frame.bbox_e0 + np.arange(n) * step
    ns = frame.bbox_n0 + np.arange(n) * step
    ee, nn = np.meshgrid(es, ns)
    ee, nn = ee.ravel(), nn.ravel()
    z = terrain.height(ee, nn) - frame.origin_h
    x, y = frame.en_to_world(ee, nn)
    verts = np.column_stack([x, y, z])
    idx = np.arange(n * n).reshape(n, n)
    a, b = idx[:-1, :-1].ravel(), idx[:-1, 1:].ravel()
    c, d = idx[1:, :-1].ravel(), idx[1:, 1:].ravel()
    faces = np.vstack([np.column_stack([a, b, d]), np.column_stack([a, d, c])])
    u, v = frame.uv(ee, nn)
    return verts, faces, np.column_stack([u, v])


# ----------------------------------------------------------------- buildings --

def _newell(pts: np.ndarray) -> np.ndarray:
    nrm = np.zeros(3)
    for i in range(len(pts)):
        a, b = pts[i], pts[(i + 1) % len(pts)]
        nrm += [(a[1] - b[1]) * (a[2] + b[2]),
                (a[2] - b[2]) * (a[0] + b[0]),
                (a[0] - b[0]) * (a[1] + b[1])]
    ln = np.linalg.norm(nrm)
    return nrm / ln if ln > 0 else nrm


def triangulate_polygon(rings: list[np.ndarray]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Triangulate a planar 3D polygon (outer ring + holes).

    Returns vertices, faces (oriented to the ring's Newell normal), normal.
    """
    outer = rings[0]
    normal = _newell(outer)
    if not normal.any():
        return np.empty((0, 3)), np.empty((0, 3), int), normal
    drop = int(np.argmax(np.abs(normal)))          # project onto dominant plane
    keep = [i for i in range(3) if i != drop]
    verts = np.vstack(rings)
    ends = np.cumsum([len(r) for r in rings]).astype(np.uint32)
    tri = earcut.triangulate_float64(verts[:, keep].astype(np.float64), ends)
    faces = np.asarray(tri, np.int64).reshape(-1, 3)
    if len(faces):
        fn = np.cross(verts[faces[:, 1]] - verts[faces[:, 0]],
                      verts[faces[:, 2]] - verts[faces[:, 0]])
        flip = (fn @ normal) < 0
        faces[flip] = faces[flip][:, ::-1]
    return verts, faces, normal


def _poslist(elem) -> np.ndarray:
    vals = np.array(elem.text.split(), float).reshape(-1, 3)
    if len(vals) > 1 and np.allclose(vals[0], vals[-1]):
        vals = vals[:-1]  # GML rings repeat the first point
    return vals


def load_buildings(raw: Path, frame: Frame):
    """Parse LoD2 buildings inside the bbox.

    Returns (roof polygons, wall polygons, 2D footprints, per-building records),
    each polygon a list of rings in UTM/DHHN2016. The records keep each
    building's own surfaces plus its ALKIS attributes (function, roof type,
    measured height) for the engine-neutral scene package.
    """
    roofs, walls, footprints, records = [], [], [], []
    e_lo, n_lo = frame.bbox_e0, frame.bbox_n0
    e_hi, n_hi = e_lo + frame.size, n_lo + frame.size
    seen = set()
    for te, tn in tiles_for_bbox(frame.bbox_e0, frame.bbox_n0, frame.size):
        path = find_tile(raw, f"LoD2_32_{te}_{tn}_1_NW.gml")
        for _, bld in etree.iterparse(str(path), tag=f"{{{NS['bldg']}}}Building",
                                      huge_tree=True):
            if bld.getparent() is not None and bld.getparent().tag.endswith("BuildingPart"):
                continue
            gid = bld.get(f"{{{NS['gml']}}}id")
            function = bld.findtext(".//bldg:function", namespaces=NS)
            roof_type = bld.findtext(".//bldg:roofType", namespaces=NS)
            height = bld.findtext(".//bldg:measuredHeight", namespaces=NS)
            surfaces = {}
            for kind in ("RoofSurface", "WallSurface", "GroundSurface"):
                polys = []
                for poly in bld.iterfind(f".//bldg:{kind}//gml:Polygon", NS):
                    rings = [_poslist(p) for p in
                             poly.iterfind("gml:exterior//gml:posList", NS)]
                    rings += [_poslist(p) for p in
                              poly.iterfind("gml:interior//gml:posList", NS)]
                    if rings and len(rings[0]) >= 3:
                        polys.append(rings)
                surfaces[kind] = polys
            bld.clear()
            allpts = [r for k in surfaces.values() for p in k for r in p]
            if not allpts or gid in seen:
                continue
            cx, cy = np.vstack(allpts)[:, :2].mean(axis=0)
            if not (e_lo <= cx <= e_hi and n_lo <= cy <= n_hi):
                continue
            seen.add(gid)
            roofs += surfaces["RoofSurface"]
            walls += surfaces["WallSurface"]
            for rings in surfaces["GroundSurface"]:
                fp = Polygon(rings[0][:, :2])
                if fp.is_valid and fp.area > 1:
                    footprints.append(fp)
            records.append({
                "id": gid, "function": function, "roof_type": roof_type,
                "height": float(height) if height else None,
                "category": facade_category(function), "source": "LoD2",
                "centroid": frame.en_to_world(cx, cy),
                "roofs": surfaces["RoofSurface"], "walls": surfaces["WallSurface"]})
    return roofs, walls, footprints, records


def building_mesh(roofs, walls, frame: Frame) -> tuple[ObjWriter, int]:
    obj = ObjWriter()
    for kind, polys in (("roof", roofs), ("wall", walls)):
        vs, fs, off = [], [], 0
        for rings in polys:
            v, f, nrm = triangulate_polygon(rings)
            if not len(f):
                continue
            if kind == "roof" and nrm[2] < 0:   # roofs must face the sky
                f = f[:, ::-1]
            if kind == "wall":
                # Walls are emitted double-sided. A wall whose ring orientation
                # is inverted in the source would otherwise be back-face
                # culled - invisible to the camera AND to the GPU LiDAR, which
                # renders the scene.
                f = np.vstack([f, f[:, ::-1]])
            vs.append(v)
            fs.append(f + off)
            off += len(v)
        if not vs:
            continue
        v = np.vstack(vs)
        f = np.vstack(fs)
        x, y = frame.en_to_world(v[:, 0], v[:, 1])
        verts = np.column_stack([x, y, v[:, 2] - frame.origin_h])
        if kind == "roof":
            u, vv = frame.uv(v[:, 0], v[:, 1])   # drape the real roof photo
            obj.add("roof", verts, f, np.column_stack([u, vv]))
        else:
            # duplicated back faces need their own vertices so normals are right
            obj.add("wall", *_split_double_sided(verts, f))
    return obj, obj.triangle_count()


def _split_double_sided(verts: np.ndarray, faces: np.ndarray):
    """Give every triangle its own vertices so both faces get flat normals."""
    tv = verts[faces].reshape(-1, 3)
    tf = np.arange(len(tv)).reshape(-1, 3)
    return tv, tf


def extra_buildings(cfg_list: list, frame: Frame, terrain: Terrain):
    """Buildings that exist on site but not in the LoD2 data.

    LoD2 is derived from the cadastre, which lags reality; anything built
    since is missing. These are traced from the (newer) orthophoto in the site
    file and extruded to a flat-roofed prism - LoD1, but correct in footprint
    and height, which is what the LiDAR sees.
    Returns roof polygons, wall polygons, footprints (UTM/DHHN2016) and
    per-building records (same shape as load_buildings').
    """
    roofs, walls, footprints, records = [], [], [], []
    for b in cfg_list or []:
        ring = np.array([frame.site_to_en(*p) for p in b["footprint"]], float)
        fp = Polygon(ring)
        if not fp.exterior.is_ccw:              # CCW seen from above = roof faces up
            ring = ring[::-1]
            fp = Polygon(ring)
        base = float(terrain.height(ring[:, 0], ring[:, 1]).min()) - 0.5
        top = base + 0.5 + float(b["height"])
        b_roofs = [[np.column_stack([ring, np.full(len(ring), top)])]]
        b_walls = []
        for i in range(len(ring)):
            a, c = ring[i], ring[(i + 1) % len(ring)]
            b_walls.append([np.array([[*a, base], [*c, base], [*c, top], [*a, top]])])
        roofs += b_roofs
        walls += b_walls
        footprints.append(fp)
        records.append({
            "id": b.get("name", f"extra_{len(records)}"), "function": None,
            "roof_type": "1000", "height": float(b["height"]),
            "category": b.get("category", "unknown"),
            "source": "traced from orthophoto (newer than cadastre)",
            "centroid": frame.en_to_world(fp.centroid.x, fp.centroid.y),
            "roofs": b_roofs, "walls": b_walls})
    return roofs, walls, footprints, records


# --------------------------------------------------------------------- trees --

def detect_trees(raw: Path, frame: Frame, terrain: Terrain, footprints,
                 cfg: dict):
    """Find individual trees in the laser scan.

    NRW's laser data has no separate vegetation class, so trees are found
    geometrically: a 1 m canopy-height model (highest non-ground return minus
    bare ground) with building footprints masked out; local maxima above
    `min_height` are tree tops.
    Also returns the cropped point cloud for the prior map.
    """
    e0, n0, size = frame.bbox_e0, frame.bbox_n0, frame.size
    xs, ys, zs, cls = [], [], [], []
    for te, tn in tiles_for_bbox(e0, n0, size):
        with laspy.open(find_tile(raw, f"3dm_32_{te}_{tn}_1_nw.laz")) as f:
            for chunk in f.chunk_iterator(4_000_000):
                x, y = np.asarray(chunk.x), np.asarray(chunk.y)
                m = (x >= e0) & (x < e0 + size) & (y >= n0) & (y < n0 + size)
                if m.any():
                    xs.append(x[m]); ys.append(y[m]); zs.append(np.asarray(chunk.z)[m])
                    cls.append(np.asarray(chunk.classification)[m])
    x, y, z, c = map(np.concatenate, (xs, ys, zs, cls))
    noise = np.isin(c, [7, 18])            # ASPRS low/high noise
    x, y, z, c = x[~noise], y[~noise], z[~noise], c[~noise]

    n = int(round(size))
    col = np.clip((x - e0).astype(int), 0, n - 1)
    row = np.clip(((n0 + size) - y).astype(int), 0, n - 1)
    above = c != 2
    chm = np.zeros((n, n), np.float32)
    np.maximum.at(chm, (row[above], col[above]),
                  (z[above] - terrain.grid[row[above], col[above]]).astype(np.float32))

    cc, rr = np.meshgrid(np.arange(n) + 0.5, np.arange(n) + 0.5)
    ce, cn = e0 + cc, (n0 + size) - rr
    if footprints:
        blocked = unary_union(footprints).buffer(cfg.get("building_buffer", 1.5))
        chm[contains_xy(blocked, ce, cn)] = 0.0

    # Zones where the laser scan is stale (e.g. a construction site when it
    # was flown): its clutter would otherwise be detected as trees.
    for zone in cfg.get("exclude_zones", []):
        poly = Polygon([frame.site_to_en(*p) for p in zone])
        chm[contains_xy(poly, ce, cn)] = 0.0

    smooth = ndimage.gaussian_filter(chm, 1.0)
    min_h = cfg.get("min_height", 4.0)
    peaks = (smooth == ndimage.maximum_filter(smooth, size=5)) & (smooth >= min_h)
    pr, pc = np.nonzero(peaks)
    h = smooth[pr, pc]
    order = np.argsort(-h)
    pr, pc, h = pr[order], pc[order], h[order]
    pts = np.column_stack([pc, pr]).astype(float)
    sep2 = cfg.get("min_separation", 4.0) ** 2
    chosen: list[int] = []
    for i, p in enumerate(pts):              # greedy non-maximum suppression
        if chosen and (((pts[chosen] - p) ** 2).sum(axis=1) < sep2).any():
            continue
        chosen.append(i)
    trees = []
    for i in chosen:
        te_, tn_ = e0 + pc[i] + 0.5, (n0 + size) - pr[i] - 0.5
        height = float(h[i])
        # crown radius: mean distance to where the canopy drops below half height
        radii = []
        for ang in np.linspace(0, 2 * np.pi, 8, endpoint=False):
            for d in range(1, 12):
                rr_ = int(pr[i] - d * math.sin(ang)); cc_ = int(pc[i] + d * math.cos(ang))
                if not (0 <= rr_ < n and 0 <= cc_ < n) or smooth[rr_, cc_] < 0.5 * height:
                    radii.append(d); break
            else:
                radii.append(11)
        trees.append((te_, tn_, height, float(np.clip(np.mean(radii), 1.2, 8.0))))
    return trees, (x, y, z, c)


def _icosphere(subdiv: int = 1):
    t = (1 + 5 ** 0.5) / 2
    v = np.array([[-1, t, 0], [1, t, 0], [-1, -t, 0], [1, -t, 0], [0, -1, t], [0, 1, t],
                  [0, -1, -t], [0, 1, -t], [t, 0, -1], [t, 0, 1], [-t, 0, -1], [-t, 0, 1]], float)
    f = np.array([[0, 11, 5], [0, 5, 1], [0, 1, 7], [0, 7, 10], [0, 10, 11], [1, 5, 9],
                  [5, 11, 4], [11, 10, 2], [10, 7, 6], [7, 1, 8], [3, 9, 4], [3, 4, 2],
                  [3, 2, 6], [3, 6, 8], [3, 8, 9], [4, 9, 5], [2, 4, 11], [6, 2, 10],
                  [8, 6, 7], [9, 8, 1]])
    for _ in range(subdiv):
        mid, nf = {}, []
        vl = list(v)
        def m(a, b):
            k = (min(a, b), max(a, b))
            if k not in mid:
                vl.append((vl[a] + vl[b]) / 2); mid[k] = len(vl) - 1
            return mid[k]
        for a, b, c in f:
            ab, bc, ca = m(a, b), m(b, c), m(c, a)
            nf += [[a, ab, ca], [b, bc, ab], [c, ca, bc], [ab, bc, ca]]
        v, f = np.array(vl), np.array(nf)
    return v / np.linalg.norm(v, axis=1, keepdims=True), f


def _cylinder(r: float, h: float, seg: int = 8):
    a = np.linspace(0, 2 * np.pi, seg, endpoint=False)
    ring = np.column_stack([np.cos(a) * r, np.sin(a) * r])
    v = np.vstack([np.column_stack([ring, np.zeros(seg)]), np.column_stack([ring, np.full(seg, h)])])
    f = []
    for i in range(seg):
        j = (i + 1) % seg
        f += [[i, j, seg + j], [i, seg + j, seg + i]]
    return v, np.array(f)


def tree_mesh(trees, frame: Frame, terrain: Terrain) -> ObjWriter:
    obj = ObjWriter()
    sph_v, sph_f = _icosphere(1)
    tv, tf, cv, cf = [], [], [], []
    toff = coff = 0
    for e, n, h, r in trees:
        gz = float(terrain.height(e, n)) - frame.origin_h
        x, y = frame.en_to_world(e, n)
        trunk_h = 0.4 * h
        v, f = _cylinder(max(0.15, 0.025 * h), trunk_h)
        tv.append(v + [x, y, gz]); tf.append(f + toff); toff += len(v)
        crown_c = gz + trunk_h + 0.5 * (h - trunk_h)
        v = sph_v * [r, r, 0.5 * (h - trunk_h) + 0.3] + [x, y, crown_c]
        cv.append(v); cf.append(sph_f + coff); coff += len(v)
    if trees:
        obj.add("trunk", np.vstack(tv), np.vstack(tf))
        obj.add("leaves", np.vstack(cv), np.vstack(cf))
    return obj


# ----------------------------------------------------------------- perimeter --

def _box(cx, cy, cz, lx, ly, lz, yaw):
    """Axis-aligned box centred at (cx,cy,cz), rotated by yaw about z."""
    dx, dy, dz = lx / 2, ly / 2, lz / 2
    c = np.array([[sx * dx, sy * dy, sz * dz] for sz in (-1, 1) for sy in (-1, 1) for sx in (-1, 1)])
    cs, sn = math.cos(yaw), math.sin(yaw)
    rot = np.array([[cs, -sn, 0], [sn, cs, 0], [0, 0, 1]])
    v = c @ rot.T + [cx, cy, cz]
    f = np.array([[0, 2, 1], [1, 2, 3], [4, 5, 6], [5, 7, 6], [0, 1, 4], [1, 5, 4],
                  [2, 6, 3], [3, 6, 7], [0, 4, 2], [2, 4, 6], [1, 3, 5], [3, 7, 5]])
    return v, f


class Parts:
    """Collects boxes per material, and remembers each one as a typed instance.

    Gazebo renders the boxes; other engines get the instance list (type, centre,
    size, yaw) and swap in their own fence/gate/floodlight assets.
    """

    def __init__(self):
        self.v: dict[str, list] = {}
        self.f: dict[str, list] = {}
        self.n: dict[str, int] = {}
        self.instances: list[dict] = []

    def box(self, mat, cx, cy, cz, lx, ly, lz, yaw):
        v, f = _box(cx, cy, cz, lx, ly, lz, yaw)
        self.v.setdefault(mat, []).append(v)
        self.f.setdefault(mat, []).append(f + self.n.get(mat, 0))
        self.n[mat] = self.n.get(mat, 0) + len(v)
        self.instances.append({
            "type": mat,
            "centre": [round(float(cx), 3), round(float(cy), 3), round(float(cz), 3)],
            "size": [round(float(lx), 3), round(float(ly), 3), round(float(lz), 3)],
            "yaw": round(float(yaw), 5)})

    def to_obj(self) -> ObjWriter:
        obj = ObjWriter()
        for mat in self.v:
            v, f = np.vstack(self.v[mat]), np.vstack(self.f[mat])
            obj.add(mat, *_split_double_sided(v, f))  # flat-shaded boxes
        return obj


def build_perimeter(cfg: dict, frame: Frame, terrain: Terrain, footprints):
    """Fence, gate, guard booth and floodlights around the secure area.

    Fence runs along the polygon; wherever it would cut through a building,
    that stretch is dropped - the building wall is the perimeter there, as on
    a real site. Configured breaches are left open on purpose (the scenario).
    """
    parts = Parts()
    ground = lambda e, n: float(terrain.height(e, n)) - frame.origin_h
    poly_en = [frame.site_to_en(*p) for p in cfg["polygon"]]
    fence_h = cfg.get("fence_height", 2.4)
    spacing = cfg.get("post_spacing", 3.0)
    blocked = unary_union(footprints).buffer(0.4) if footprints else None
    gaps = []
    for g in cfg.get("gates", []):
        gaps.append((Point(frame.site_to_en(*g["at"])), g.get("width", 6.0) / 2, "gate", g))
    for b in cfg.get("breaches", []):
        gaps.append((Point(frame.site_to_en(*b["at"])), b.get("width", 3.0) / 2, "breach", b))

    fence_lines = []
    n_panels = n_skipped = 0
    for i in range(len(poly_en)):
        a, b = np.array(poly_en[i]), np.array(poly_en[(i + 1) % len(poly_en)])
        length = float(np.linalg.norm(b - a))
        yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        steps = max(1, int(round(length / spacing)))
        for s in range(steps):
            p0 = a + (b - a) * s / steps
            p1 = a + (b - a) * (s + 1) / steps
            mid = (p0 + p1) / 2
            seg = LineString([tuple(p0), tuple(p1)])
            if blocked is not None and seg.intersects(blocked):
                n_skipped += 1
                continue
            if any(Point(*mid).distance(pt) < half for pt, half, _, _ in gaps):
                continue
            seg_len = float(np.linalg.norm(p1 - p0))
            gz = ground(*mid)
            x, y = frame.en_to_world(*mid)
            # steel palisade panel + top rail + post
            parts.box("fence", x, y, gz + fence_h / 2, seg_len, 0.06, fence_h, yaw)
            parts.box("post", *frame.en_to_world(*p0), ground(*p0) + (fence_h + 0.2) / 2,
                      0.1, 0.1, fence_h + 0.2, yaw)
            fence_lines.append((p0, p1))
            n_panels += 1

    for pt, half, kind, g in gaps:
        if kind != "gate":
            continue
        e, n = pt.x, pt.y
        yaw = math.radians(g.get("yaw_deg", 0.0))
        x, y = frame.en_to_world(e, n)
        gz = ground(e, n)
        # two gate posts and a closed sliding gate leaf
        for side in (-1, 1):
            px_, py_ = x + side * half * math.cos(yaw), y + side * half * math.sin(yaw)
            parts.box("post", px_, py_, gz + 1.4, 0.25, 0.25, 2.8, yaw)
        parts.box("gate", x, y, gz + 1.1, 2 * half, 0.08, 2.0, yaw)

    if "guard_booth" in cfg:
        gb = cfg["guard_booth"]
        e, n = frame.site_to_en(*gb["at"])
        x, y = frame.en_to_world(e, n)
        gz = ground(e, n)
        yaw = math.radians(gb.get("yaw_deg", 0.0))
        parts.box("booth", x, y, gz + 1.3, 3.0, 2.5, 2.6, yaw)
        parts.box("booth_roof", x, y, gz + 2.7, 3.4, 2.9, 0.2, yaw)
        parts.box("window", x, y, gz + 1.7, 3.02, 2.52, 0.8, yaw)

    light_every = cfg.get("light_spacing", 30.0)
    ring = LineString(poly_en + [poly_en[0]])
    n_lights = int(ring.length // light_every)
    cen = Polygon(poly_en).centroid
    for k in range(n_lights):
        p = ring.interpolate(k * light_every)
        # step 1 m inwards so poles stand inside the fence
        d = np.array([cen.x - p.x, cen.y - p.y]); d /= max(np.linalg.norm(d), 1e-6)
        e, n = p.x + d[0], p.y + d[1]
        if blocked is not None and blocked.contains(Point(e, n)):
            continue
        x, y = frame.en_to_world(e, n)
        gz = ground(e, n)
        parts.box("pole", x, y, gz + 3.5, 0.15, 0.15, 7.0, 0.0)
        parts.box("lamp", x + 0.3 * d[0], y + 0.3 * d[1], gz + 6.9, 0.6, 0.3, 0.15, 0.0)

    print(f"  perimeter: {n_panels} fence panels, {n_skipped} skipped where a "
          f"building wall forms the boundary, {len([g for g in gaps if g[2]=='gate'])} gate(s), "
          f"{len([g for g in gaps if g[2]=='breach'])} breach(es), {n_lights} floodlight poles")
    return parts.to_obj(), fence_lines, parts.instances


# -------------------------------------------------------------------- actors --

WALK_DAE = "https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae"


def actor_waypoints(path_site: list, frame: Frame, terrain: Terrain,
                    speed: float = 1.2) -> list[dict]:
    """Timed waypoints of a looped walk: {t, x, y, z_ground, yaw} in world ENU.

    Computed once and used by both Gazebo (actor script) and the scene
    manifest, so every engine places a walking person at the same spot at the
    same simulation time. Each leg is written as a start and an end waypoint
    with the leg's heading, so the person turns on the spot at corners.
    """
    pts = [frame.site_to_en(*p) for p in path_site]
    pts.append(pts[0])
    t, wps = 0.0, []
    for i in range(len(pts) - 1):
        (e0, n0), (e1, n1) = pts[i], pts[i + 1]
        yaw = math.atan2(n1 - n0, e1 - e0)
        leg = math.hypot(e1 - e0, n1 - n0) / speed
        for (e, n), tt in (((e0, n0), t), ((e1, n1), t + leg)):
            x, y = frame.en_to_world(e, n)
            wps.append({"t": round(tt, 3), "x": round(float(x), 3), "y": round(float(y), 3),
                        "z_ground": round(float(terrain.height(e, n)) - frame.origin_h, 3),
                        "yaw": round(yaw, 5)})
        t += leg + 0.001
    return wps


def actor_sdf(name: str, wps: list[dict]) -> str:
    """An animated walking person following a looped path (Gazebo actor)."""
    xml = "".join(
        f"<waypoint><time>{w['t']:.3f}</time><pose>{w['x']:.3f} {w['y']:.3f} "
        f"{w['z_ground'] + 1.0:.3f} 0 0 {w['yaw']:.5f}</pose></waypoint>"  # +1 m: mesh root is the hip
        for w in wps)
    return f"""
    <actor name="{name}">
      <skin><filename>{WALK_DAE}</filename><scale>1.0</scale></skin>
      <animation name="walk"><filename>{WALK_DAE}</filename><interpolate_x>true</interpolate_x></animation>
      <script><loop>true</loop><auto_start>true</auto_start>
        <trajectory id="0" type="walk">
          {xml}
        </trajectory>
      </script>
    </actor>"""


# ----------------------------------------------------------------- the world --

def world_sdf(name: str, frame: Frame, lat: float, lon: float, actors: str) -> str:
    plugins = "\n".join(f'    <plugin filename="{f}" name="{n}"/>' for f, n in PX4_SYSTEMS)
    def static(model, visual_obj, collision_obj):
        return f"""
    <model name="{model}">
      <static>true</static>
      <link name="link">
        <visual name="visual"><geometry><mesh><uri>meshes/{visual_obj}</uri></mesh></geometry></visual>
        <collision name="collision"><geometry><mesh><uri>meshes/{collision_obj}</uri></mesh></geometry></collision>
      </link>
    </model>"""
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- Generated by scripts/build_real_site.py from NRW OpenGeodata
     (Land NRW, dl-de/zero-2.0). Do not edit by hand; edit the site YAML and rebuild.
     World origin = launch pad at E={frame.origin_e:.1f} N={frame.origin_n:.1f} (EPSG:25832),
     heights relative to ground there ({frame.origin_h:.2f} m DHHN2016). -->
<sdf version="1.9">
  <world name="{name}">
    <physics name="4ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
      <!-- large static meshes: bullet collision detection is far more robust
           than the default ODE detector for triangle meshes -->
      <dart><collision_detector>bullet</collision_detector></dart>
    </physics>
{plugins}
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.62 0.74 0.88 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
    </scene>
    <light name="sun" type="directional">
      <pose>0 0 300 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <diffuse>0.95 0.93 0.88 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.3 -0.85</direction>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{lat:.9f}</latitude_deg>
      <longitude_deg>{lon:.9f}</longitude_deg>
      <elevation>{frame.origin_h:.2f}</elevation>
    </spherical_coordinates>
{static("terrain", "terrain.obj", "terrain_collision.obj")}
{static("buildings", "buildings.obj", "buildings.obj")}
{static("trees", "trees.obj", "trees.obj")}
{static("perimeter", "perimeter.obj", "perimeter.obj")}
{actors}
  </world>
</sdf>
"""


# ---------------------------------------------------------------------- main --

def write_preview(path: Path, ortho: Image.Image, frame: Frame, footprints, trees,
                  fence_lines, cfg: dict):
    im = ortho.copy()
    if im.width > 1500:
        im = im.resize((1500, 1500))
    s = im.width / frame.size
    d = ImageDraw.Draw(im)
    def px(e, n):
        return ((e - frame.bbox_e0) * s, (frame.bbox_n0 + frame.size - n) * s)
    for k in range(0, int(frame.size) + 1, 50):
        p = k * s
        col = (255, 255, 0) if k % 100 == 0 else (160, 160, 0)
        d.line([(p, 0), (p, im.height)], fill=col, width=1)
        d.line([(0, p), (im.width, p)], fill=col, width=1)
        if k % 100 == 0:
            d.text((p + 3, 3), f"x{k - frame.size/2:+.0f}", fill=(255, 255, 0))
            d.text((3, p + 3), f"y{frame.size/2 - k:+.0f}", fill=(255, 255, 0))
    for fp in footprints:
        d.line([px(*c) for c in fp.exterior.coords], fill=(255, 60, 60), width=1)
    for b in cfg.get("extra_buildings", []) or []:
        pts = [px(*frame.site_to_en(*p)) for p in b["footprint"]]
        d.line(pts + [pts[0]], fill=(255, 150, 0), width=3)
    for z in cfg.get("exclude_trees", []) or []:
        pts = [px(*frame.site_to_en(*p)) for p in z]
        d.line(pts + [pts[0]], fill=(200, 200, 200), width=1)
    for e, n, h, r in trees:
        cx, cy = px(e, n)
        d.ellipse([cx - r * s, cy - r * s, cx + r * s, cy + r * s], outline=(60, 255, 60))
    poly = [px(*frame.site_to_en(*p)) for p in cfg["perimeter"]["polygon"]]
    d.line(poly + [poly[0]], fill=(255, 255, 255), width=1)
    for p0, p1 in fence_lines:
        d.line([px(*p0), px(*p1)], fill=(0, 200, 255), width=4)
    for g in cfg["perimeter"].get("gates", []):
        cx, cy = px(*frame.site_to_en(*g["at"]))
        d.rectangle([cx - 5, cy - 5, cx + 5, cy + 5], fill=(0, 255, 0)); d.text((cx + 7, cy - 7), "GATE", fill=(0, 255, 0))
    for b in cfg["perimeter"].get("breaches", []):
        cx, cy = px(*frame.site_to_en(*b["at"]))
        d.ellipse([cx - 6, cy - 6, cx + 6, cy + 6], outline=(255, 0, 0), width=3); d.text((cx + 8, cy - 7), "BREACH", fill=(255, 0, 0))
    if "guard_booth" in cfg["perimeter"]:
        cx, cy = px(*frame.site_to_en(*cfg["perimeter"]["guard_booth"]["at"]))
        d.rectangle([cx - 6, cy - 5, cx + 6, cy + 5], fill=(255, 140, 0)); d.text((cx + 8, cy - 7), "GUARD", fill=(255, 140, 0))
    for a in cfg.get("actors", []):
        pts = [px(*frame.site_to_en(*p)) for p in a["path"]]
        d.line(pts + [pts[0]], fill=(255, 0, 255), width=2); d.text(pts[0], a["name"], fill=(255, 0, 255))
    cx, cy = px(frame.origin_e, frame.origin_n)
    d.ellipse([cx - 8, cy - 8, cx + 8, cy + 8], outline=(255, 0, 255), width=3); d.text((cx + 10, cy + 2), "LAUNCH PAD (world 0,0)", fill=(255, 0, 255))
    im.save(path)


def export_scene(out: Path, cfg: dict, raw: Path, frame: Frame, terrain: Terrain,
                 lat: float, lon: float, roof_ortho: Image.Image, bld_records: list,
                 trees: list, perim_parts: list, actor_paths: list) -> Path:
    """Write the engine-neutral scene package (see scene_package.py)."""
    import scene_package as sp

    sc = cfg.get("scene", {})
    tex_dir = out / "scene" / "textures"
    tex_dir.mkdir(parents=True, exist_ok=True)

    print("[+] scene package: 10 cm orthophoto tiles")
    ortho_full = build_orthophoto(raw, frame.bbox_e0, frame.bbox_n0, frame.size,
                                  sc.get("texture_m_per_px", 0.1))
    terrain_meshes, tile_records = sp.terrain_tiles(
        terrain, frame, ortho_full, sc.get("tile_m", 150.0),
        sc.get("terrain_step", 1.0), tex_dir)
    del ortho_full

    print("[+] scene package: vegetation (NDVI) from the photo's infrared band")
    ndvi_px = sc.get("ndvi_m_per_px", 0.4)
    rgb, nir = build_orthophoto(raw, frame.bbox_e0, frame.bbox_n0, frame.size, ndvi_px,
                                with_nir=True)
    ndvi = sp.ndvi_image(rgb.split()[0], nir)
    veg = (np.asarray(ndvi) > (0.3 + 1) * 127.5).mean()

    print("[+] scene package: buildings, instances, manifest")
    roof_tex = sp._jpeg(roof_ortho, tex_dir / "roofs.jpg")
    building_parts = sp.building_meshes(bld_records, frame, triangulate_polygon, roof_tex)
    trees_world = []
    for e, n, h, r in trees:
        x, y = frame.en_to_world(e, n)
        trees_world.append([x, y, float(terrain.height(e, n)) - frame.origin_h, h, r])
    counts = {}
    for p in perim_parts:
        counts[p["type"]] = counts.get(p["type"], 0) + 1
    stats = {"buildings": len(bld_records), "trees": len(trees_world),
             "terrain_tiles": len(tile_records),
             "perimeter_parts": counts,
             "vegetation_fraction": round(float(veg), 3),
             "triangles": {name: int(len(m.faces))
                           for name, m in terrain_meshes[:1] + building_parts}}
    path = sp.write_package(out, cfg=cfg, frame=frame, lat=lat, lon=lon,
                            terrain_meshes=terrain_meshes, tile_records=tile_records,
                            building_parts=building_parts, building_records=bld_records,
                            trees_world=trees_world, perimeter_parts=perim_parts,
                            actors=actor_paths, ndvi=ndvi, ndvi_m_per_px=ndvi_px,
                            stats=stats)
    try:
        import jsonschema
        schema = json.loads((Path(__file__).parent / "scene_manifest.schema.json").read_text())
        jsonschema.validate(json.loads(path.read_text()), schema)
        print("      manifest valid against scripts/scene_manifest.schema.json")
    except ImportError:
        print("      (jsonschema not installed - manifest not validated)")
    sizes = {p.name: p.stat().st_size / 1e6 for p in (out / "scene").glob("*.*")}
    print(f"      {len(tile_records)} terrain tiles, {len(bld_records)} buildings, "
          f"{len(trees_world)} trees, {len(perim_parts)} perimeter parts, "
          f"{len(actor_paths)} actors, vegetation {veg:.0%} of area")
    print("      " + ", ".join(f"{k} {v:.1f} MB" for k, v in sorted(sizes.items())))
    print(f"      -> {path}")
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("site", type=Path, help="site YAML")
    ap.add_argument("--preview-only", action="store_true",
                    help="only render preview.png (fast iteration on the layout)")
    ap.add_argument("--export-scene", action="store_true",
                    help="also write the engine-neutral scene package (glTF + manifest) "
                         "used by Unity now and Isaac Sim later")
    args = ap.parse_args()
    cfg = yaml.safe_load(args.site.read_text())
    raw = Path(os.path.expanduser(cfg["raw_dir"]))
    out = Path(os.path.expanduser(cfg["out_dir"]))
    (out / "meshes").mkdir(parents=True, exist_ok=True)

    b = cfg["bbox"]
    frame = Frame(b["e0"], b["n0"], b["size"], 0, 0, 0)
    print(f"[1/7] terrain  (bbox {b['size']} m at E{b['e0']} N{b['n0']})")
    terrain = Terrain(raw, frame.bbox_e0, frame.bbox_n0, frame.size)
    frame.origin_e, frame.origin_n = frame.site_to_en(*cfg["launch_pad"])
    frame.origin_h = float(terrain.height(frame.origin_e, frame.origin_n))
    to_ll = Transformer.from_crs("EPSG:25832", "EPSG:4326", always_xy=True)
    lon, lat = to_ll.transform(frame.origin_e, frame.origin_n)
    print(f"      launch pad {lat:.6f}N {lon:.6f}E, ground {frame.origin_h:.2f} m, "
          f"terrain {np.nanmin(terrain.grid):.1f}-{np.nanmax(terrain.grid):.1f} m")

    print("[2/7] orthophoto")
    ortho = build_orthophoto(raw, frame.bbox_e0, frame.bbox_n0, frame.size,
                             cfg.get("texture_m_per_px", 0.2))
    print(f"      {ortho.width}x{ortho.height} px")

    print("[3/7] buildings (LoD2)")
    roofs, walls, footprints, bld_records = load_buildings(raw, frame)
    print(f"      {len(footprints)} footprints, {len(roofs)} roof / {len(walls)} wall polygons")
    xr, xw, xf, x_records = extra_buildings(cfg.get("extra_buildings"), frame, terrain)
    roofs += xr; walls += xw; footprints += xf; bld_records += x_records
    if xf:
        print(f"      + {len(xf)} building(s) newer than the cadastre, traced from the photo")

    print("[4/7] trees from laser scan")
    tcfg = dict(cfg.get("trees", {}), exclude_zones=cfg.get("exclude_trees", []))
    trees, cloud = detect_trees(raw, frame, terrain, footprints, tcfg)
    print(f"      {len(trees)} trees, {len(cloud[0]):,} laser points in bbox")

    print("[5/7] security perimeter")
    perim_obj, fence_lines, perim_parts = build_perimeter(cfg["perimeter"], frame, terrain,
                                                          footprints)

    write_preview(out / "preview.png", ortho, frame, footprints, trees, fence_lines, cfg)
    print(f"      preview -> {out / 'preview.png'}")
    if args.preview_only:
        return

    print("[6/7] meshes")
    ortho.save(out / "meshes" / "ortho.jpg", quality=90)
    step = cfg.get("terrain_mesh_step", 2.0)
    obj = ObjWriter()
    gv, gf, guv = grid_mesh(terrain, frame, step)
    obj.add("ground", gv, gf, guv)
    obj.write(out / "meshes" / "terrain.obj", {"ground": {"kd": (1, 1, 1), "map_kd": "ortho.jpg"}})
    col = ObjWriter()
    cv, cf, _ = grid_mesh(terrain, frame, cfg.get("terrain_collision_step", 6.0))
    col.add("ground", cv, cf)
    col.write(out / "meshes" / "terrain_collision.obj", {"ground": {}})
    bobj, btris = building_mesh(roofs, walls, frame)
    bobj.write(out / "meshes" / "buildings.obj",
               {"roof": {"kd": (1, 1, 1), "map_kd": "ortho.jpg"},
                "wall": {"kd": (0.78, 0.75, 0.70)}})
    tobj = tree_mesh(trees, frame, terrain)
    tobj.write(out / "meshes" / "trees.obj",
               {"trunk": {"kd": (0.35, 0.25, 0.15)}, "leaves": {"kd": (0.22, 0.42, 0.18)}})
    perim_obj.write(out / "meshes" / "perimeter.obj", {
        "fence": {"kd": (0.18, 0.30, 0.22)}, "post": {"kd": (0.2, 0.2, 0.2)},
        "gate": {"kd": (0.75, 0.62, 0.10)}, "booth": {"kd": (0.85, 0.85, 0.82)},
        "booth_roof": {"kd": (0.25, 0.25, 0.28)}, "window": {"kd": (0.25, 0.35, 0.45)},
        "pole": {"kd": (0.5, 0.5, 0.5)}, "lamp": {"kd": (1.0, 0.95, 0.7)}})
    print(f"      triangles: terrain {obj.triangle_count():,}, buildings {btris:,}, "
          f"trees {tobj.triangle_count():,}, perimeter {perim_obj.triangle_count():,}")

    print("[7/7] world + prior map")
    actor_paths = [{"name": a["name"], "speed": a.get("speed", 1.2), "loop": True,
                    "waypoints": actor_waypoints(a["path"], frame, terrain, a.get("speed", 1.2))}
                   for a in cfg.get("actors", [])]
    actors = "".join(actor_sdf(a["name"], a["waypoints"]) for a in actor_paths)
    (out / f"{cfg['name']}.sdf").write_text(world_sdf(cfg["name"], frame, lat, lon, actors))

    # Prior map = real laser scan + the perimeter the operator built (sampled),
    # in world coordinates. Walking intruders are deliberately absent.
    x, y, z, c = cloud
    wx, wy = frame.en_to_world(x, y)
    fence_pts = []
    for p0, p1 in fence_lines:
        for t in np.linspace(0, 1, 12):
            e, n = p0 + (p1 - p0) * t
            gz = float(terrain.height(e, n))
            for hz in np.linspace(0.1, cfg["perimeter"].get("fence_height", 2.4), 8):
                fence_pts.append((e, n, gz + hz))
    fp_ = np.array(fence_pts) if fence_pts else np.empty((0, 3))
    fx, fy = frame.en_to_world(fp_[:, 0], fp_[:, 1])
    hdr = laspy.LasHeader(point_format=6, version="1.4")  # fmt 6: 8-bit classes
    hdr.scales = [0.01, 0.01, 0.01]
    hdr.offsets = [0, 0, 0]
    las = laspy.LasData(hdr)
    las.x = np.concatenate([wx, fx])
    las.y = np.concatenate([wy, fy])
    las.z = np.concatenate([z - frame.origin_h, fp_[:, 2] - frame.origin_h])
    las.classification = np.concatenate([c, np.full(len(fp_), 64, np.uint8)])  # 64 = designed perimeter
    las.write(out / "prior_map.laz")
    print(f"      world -> {out / (cfg['name'] + '.sdf')}")
    print(f"      prior -> {out / 'prior_map.laz'} ({len(las.x):,} points)")

    if args.export_scene:
        export_scene(out, cfg, raw, frame, terrain, lat, lon, ortho, bld_records, trees,
                     perim_parts, actor_paths)
    print("\nRun it:\n"
          f"  ./start_uav_sim.sh --world {out / (cfg['name'] + '.sdf')} "
          "--model gz_x500_threat_scanner --sensors")


if __name__ == "__main__":
    main()
