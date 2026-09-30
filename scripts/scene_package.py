"""Engine-neutral scene package: glTF meshes + a JSON manifest.

Gazebo gets its world from build_real_site.py directly. Every *other* engine -
Unity now, Isaac Sim later - builds its scene from this package instead, so
all engines render the same real site and nobody hand-edits a scene.

Two rules shape the format:

1. Static geometry that is the same everywhere (terrain, buildings) is shipped
   as glTF 2.0 (.glb). Unity loads it with glTFast; Isaac converts it to USD.
2. Everything an engine should render with its *own* asset is an instance in
   the manifest (trees, fence panels, floodlights, people): position, size,
   yaw. Gazebo draws a tree as an ellipsoid, Unity places a realistic tree
   prefab of the same height and crown, Isaac uses its own. The positions
   come from the real laser scan either way.

Coordinates
-----------
Manifest: world ENU metres, identical to the Gazebo world - x = East,
y = North, z = Up, origin at the launch pad (PX4 spawn point), heights relative
to the ground there. Yaw is counter-clockwise from East, radians (ROS REP 103).

glTF files: glTF is right-handed with +Y up, so meshes are written as
    glTF (X, Y, Z) = (East, Up, -North)
which is a pure rotation of ENU (no mirroring). Each engine converts from there
exactly once (Unity: see docs/UNITY_BRIDGE.md).
"""

from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import trimesh
from PIL import Image

MANIFEST_VERSION = 1

# ALKIS building-function codes (Gebaeudefunktion, 31001_xxxx) grouped into the
# categories a renderer needs to choose a facade material. The leading digit
# of the code is the ALKIS main group: 1 residential, 2 commerce/industry,
# 3 public. 51009 is "other structure" (canopies, carports...). The raw code
# is kept in the manifest so finer mappings can be added later.
def facade_category(function: str | None) -> str:
    if not function or "_" not in function:
        return "unknown"
    group, code = function.split("_", 1)
    if group == "31001" and code:
        return {"1": "residential", "2": "commercial", "3": "public"}.get(code[0], "other")
    if group == "51009":
        return "structure"
    return "other"


def enu_to_gltf(v: np.ndarray) -> np.ndarray:
    """World ENU (x=E, y=N, z=U) -> glTF (X=E, Y=U, Z=-N). A rotation, det +1."""
    v = np.asarray(v, float)
    return np.column_stack([v[:, 0], v[:, 2], -v[:, 1]])


def _pbr(name: str, *, texture: Image.Image | None = None,
         color=(200, 200, 200), double_sided: bool = False,
         roughness: float = 0.9):
    kw = dict(name=name, metallicFactor=0.0, roughnessFactor=roughness,
              doubleSided=double_sided)
    if texture is not None:
        kw["baseColorTexture"] = texture
    else:
        kw["baseColorFactor"] = [*color, 255]
    return trimesh.visual.material.PBRMaterial(**kw)


def _jpeg(img: Image.Image, path: Path, quality: int = 88) -> Image.Image:
    """Save as JPEG and reopen, so trimesh embeds the JPEG bytes (not a PNG)."""
    img.save(path, quality=quality)
    return Image.open(path)


# ------------------------------------------------------------------- terrain --

def terrain_tiles(terrain, frame, ortho_full: Image.Image, tile_m: float,
                  step: float, tex_dir: Path):
    """Terrain split into square tiles, each with its own 10 cm photo texture.

    One texture cannot hold 10 cm over 600 m (6000 px and more), and tiling also
    lets an engine load only the area it needs (e.g. just the compound).
    Returns (list of (name, trimesh), list of tile records for the manifest).
    """
    px_per_m = ortho_full.width / frame.size
    n_tiles = int(math.ceil(frame.size / tile_m))
    meshes, records = [], []
    for ti in range(n_tiles):            # east
        for tj in range(n_tiles):        # north
            e0 = frame.bbox_e0 + ti * tile_m
            n0 = frame.bbox_n0 + tj * tile_m
            size = min(tile_m, frame.bbox_e0 + frame.size - e0)
            k = int(round(size / step)) + 1
            es = e0 + np.linspace(0, size, k)
            ns = n0 + np.linspace(0, size, k)
            ee, nn = np.meshgrid(es, ns)
            ee, nn = ee.ravel(), nn.ravel()
            x, y = frame.en_to_world(ee, nn)
            z = terrain.height(ee, nn) - frame.origin_h
            idx = np.arange(k * k).reshape(k, k)
            a, b = idx[:-1, :-1].ravel(), idx[:-1, 1:].ravel()
            c, d = idx[1:, :-1].ravel(), idx[1:, 1:].ravel()
            # counter-clockwise seen from above in ENU -> normals point up
            faces = np.vstack([np.column_stack([a, b, d]), np.column_stack([a, d, c])])
            # OpenGL-style UVs (v = 0 at the south edge); trimesh flips V to the
            # glTF convention on export.
            uv = np.column_stack([(ee - e0) / size, (nn - n0) / size])
            left = int(round((e0 - frame.bbox_e0) * px_per_m))
            top = int(round((frame.bbox_n0 + frame.size - (n0 + size)) * px_per_m))
            span = int(round(size * px_per_m))
            tex = _jpeg(ortho_full.crop((left, top, left + span, top + span)),
                        tex_dir / f"terrain_{ti}_{tj}.jpg")
            name = f"terrain_{ti}_{tj}"
            mesh = trimesh.Trimesh(enu_to_gltf(np.column_stack([x, y, z])), faces,
                                   process=False)
            mesh.visual = trimesh.visual.TextureVisuals(
                uv=uv, material=_pbr(name, texture=tex, roughness=1.0))
            meshes.append((name, mesh))
            wx0, wy0 = frame.en_to_world(e0, n0)
            records.append({"name": name,
                            "min": [round(float(wx0), 3), round(float(wy0), 3)],
                            "size_m": round(float(size), 3),
                            "texture_px": span})
    return meshes, records


# ----------------------------------------------------------------- buildings --

def building_meshes(records: list[dict], frame, triangulate, roof_tex: Image.Image):
    """Roofs (real aerial photo) + walls grouped by facade category.

    Walls are single-sided geometry with a double-sided material - glTF's
    proper way, instead of the duplicated triangles the Gazebo OBJ needs.
    """
    roof_v, roof_f, roof_uv, off = [], [], [], 0
    walls: dict[str, tuple[list, list, int]] = {}
    for rec in records:
        for rings in rec["roofs"]:
            v, f, nrm = triangulate(rings)
            if not len(f):
                continue
            if nrm[2] < 0:
                f = f[:, ::-1]
            u, vv = frame.uv(v[:, 0], v[:, 1])
            roof_v.append(v); roof_f.append(f + off); roof_uv.append(np.column_stack([u, vv]))
            off += len(v)
        cat = rec["category"]
        vs, fs, o = walls.get(cat, ([], [], 0))
        for rings in rec["walls"]:
            v, f, _ = triangulate(rings)
            if len(f):
                vs.append(v); fs.append(f + o); o += len(v)
        walls[cat] = (vs, fs, o)

    def to_world(v):
        x, y = frame.en_to_world(v[:, 0], v[:, 1])
        return enu_to_gltf(np.column_stack([x, y, v[:, 2] - frame.origin_h]))

    out = []
    if roof_v:
        m = trimesh.Trimesh(to_world(np.vstack(roof_v)), np.vstack(roof_f), process=False)
        m.visual = trimesh.visual.TextureVisuals(
            uv=np.vstack(roof_uv), material=_pbr("roof", texture=roof_tex, roughness=0.85))
        out.append(("roofs", m))
    wall_colors = {"residential": (214, 202, 184), "commercial": (190, 190, 186),
                   "public": (206, 204, 196), "structure": (150, 150, 150),
                   "other": (200, 196, 188), "unknown": (200, 196, 188)}
    for cat, (vs, fs, _) in sorted(walls.items()):
        if not vs:
            continue
        m = trimesh.Trimesh(to_world(np.vstack(vs)), np.vstack(fs), process=False)
        m.visual = trimesh.visual.TextureVisuals(
            material=_pbr(f"wall_{cat}", color=wall_colors.get(cat, (200, 200, 200)),
                          double_sided=True))
        out.append((f"walls_{cat}", m))
    return out


# ---------------------------------------------------------------------- NDVI --

def ndvi_image(red: Image.Image, nir: Image.Image) -> Image.Image:
    """NDVI = (NIR - R) / (NIR + R), stored as 8-bit: value = (NDVI + 1) * 127.5.

    Healthy vegetation reflects strongly in near-infrared, so NDVI > ~0.3 marks
    grass and foliage in the real 2025 photo - used to place grass only where
    grass actually is.
    """
    r = np.asarray(red, np.float32)
    n = np.asarray(nir, np.float32)
    ndvi = (n - r) / np.maximum(n + r, 1.0)
    return Image.fromarray(np.clip((ndvi + 1.0) * 127.5, 0, 255).astype(np.uint8), "L")


# ------------------------------------------------------------------ manifest --

def _r(v, nd=3):
    return [round(float(a), nd) for a in v]


def write_package(out: Path, *, cfg: dict, frame, lat: float, lon: float,
                  terrain_meshes, tile_records, building_parts, building_records,
                  trees_world, perimeter_parts, actors, ndvi: Image.Image,
                  ndvi_m_per_px: float, stats: dict) -> Path:
    """Write scene/terrain.glb, scene/buildings.glb, scene/ndvi.png, manifest."""
    scene_dir = out / "scene"
    scene_dir.mkdir(parents=True, exist_ok=True)

    def glb(path: Path, parts):
        sc = trimesh.Scene()
        for name, mesh in parts:
            sc.add_geometry(mesh, node_name=name, geom_name=name)
        path.write_bytes(sc.export(file_type="glb"))

    glb(scene_dir / "terrain.glb", terrain_meshes)
    glb(scene_dir / "buildings.glb", building_parts)
    ndvi.save(scene_dir / "ndvi.png")

    poly = cfg["perimeter"]["polygon"]
    comp = np.array([frame.site_to_world(*p) for p in poly])
    margin = 25.0
    manifest = {
        "manifest_version": MANIFEST_VERSION,
        "generator": "scripts/build_real_site.py --export-scene",
        "generated_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "site": cfg["name"],
        "license": "NRW OpenGeodata, Land NRW, dl-de/zero-2.0",
        "frame": {
            "convention": "ENU metres: x=East, y=North, z=Up; yaw CCW from East (rad)",
            "origin": "launch pad (PX4 spawn); z relative to ground there",
            "gltf_axes": "glTF X=East, Y=Up, Z=-North (right-handed, y-up)",
            "crs": "EPSG:25832 + DHHN2016",
            "origin_utm": _r([frame.origin_e, frame.origin_n, frame.origin_h], 3),
            "origin_wgs84": {"lat": round(lat, 9), "lon": round(lon, 9),
                             "elevation_m": round(frame.origin_h, 3)},
            "bbox_world": {"min": _r(frame.en_to_world(frame.bbox_e0, frame.bbox_n0)),
                           "size_m": frame.size},
        },
        "regions": {
            "full": {"min": _r(frame.en_to_world(frame.bbox_e0, frame.bbox_n0)),
                     "max": _r(frame.en_to_world(frame.bbox_e0 + frame.size,
                                                 frame.bbox_n0 + frame.size))},
            "compound": {"min": _r(comp.min(axis=0) - margin),
                         "max": _r(comp.max(axis=0) + margin)},
        },
        "sources": {"orthophoto": "DOP10 2025", "terrain": "DGM1 2021",
                    "buildings": "LoD2 (cadastre 2023)", "laser": "3D-Messdaten 2019-2024"},
        "assets": {
            "terrain": {"file": "terrain.glb", "tiles": tile_records},
            "buildings": {"file": "buildings.glb",
                          "nodes": [name for name, _ in building_parts]},
            "ndvi": {"file": "ndvi.png", "m_per_px": ndvi_m_per_px,
                     "encoding": "value = (NDVI + 1) * 127.5",
                     "vegetation_threshold_ndvi": 0.3,
                     "covers": "full bbox, row 0 = north edge"},
        },
        "buildings": [
            {"id": b["id"], "category": b["category"], "function": b["function"],
             "roof_type": b["roof_type"], "height_m": b["height"],
             "centroid": _r(b["centroid"]), "source": b["source"]}
            for b in building_records],
        "trees": [_r(t) for t in trees_world],
        "tree_format": "[x, y, z_ground, height_m, crown_radius_m] from the laser scan",
        "perimeter": perimeter_parts,
        "perimeter_format": "box parts: type, centre [x,y,z], size [length,width,height], yaw",
        "actors": actors,
        "launch_pad": [0.0, 0.0, 0.0],
        "camera": cfg.get("camera", {}),
        "sun": cfg.get("sun", {}),
        "stats": stats,
    }
    path = scene_dir / "scene_manifest.json"
    path.write_text(json.dumps(manifest, indent=1))
    return path
