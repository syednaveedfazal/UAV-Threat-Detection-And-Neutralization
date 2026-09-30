"""Tests for the engine-neutral scene package.

The unit tests always run. The package tests check a real build and are
skipped until one exists:

    ~/.venvs/geo/bin/python scripts/build_real_site.py sites/bonn_poppelsdorf.yaml --export-scene
    ~/.venvs/geo/bin/python -m pytest scripts/tests -q
"""

from __future__ import annotations

import json
import os
import re
import struct
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

SCRIPTS = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SCRIPTS))

import scene_package as sp  # noqa: E402

REPO = SCRIPTS.parent
SITE = REPO / "sites" / "bonn_poppelsdorf.yaml"


def _out_dir() -> Path:
    cfg = yaml.safe_load(SITE.read_text())
    return Path(os.path.expanduser(cfg["out_dir"]))


SCENE = _out_dir() / "scene"
needs_build = pytest.mark.skipif(not (SCENE / "scene_manifest.json").exists(),
                                 reason="no scene package built yet")


# ---------------------------------------------------------------- unit tests --

def test_enu_to_gltf_axes():
    """East -> +X, Up -> +Y, North -> -Z, and it is a rotation (no mirroring)."""
    out = sp.enu_to_gltf(np.eye(3))
    np.testing.assert_allclose(out[0], [1, 0, 0])    # East
    np.testing.assert_allclose(out[1], [0, 0, -1])   # North
    np.testing.assert_allclose(out[2], [0, 1, 0])    # Up
    assert np.isclose(np.linalg.det(out), 1.0)


@pytest.mark.parametrize("code,expected", [
    ("31001_1000", "residential"), ("31001_1100", "residential"),
    ("31001_2000", "commercial"), ("31001_3023", "public"),
    ("51009_1610", "structure"), ("99999_1", "other"),
    (None, "unknown"), ("", "unknown")])
def test_facade_category(code, expected):
    assert sp.facade_category(code) == expected


# -------------------------------------------------------------- glb reading --

def _read_glb(path: Path):
    data = path.read_bytes()
    assert data[:4] == b"glTF"
    jlen = struct.unpack_from("<I", data, 12)[0]
    gltf = json.loads(data[20:20 + jlen])
    bin_off = 20 + jlen + 8
    return gltf, data[bin_off:]


def _accessor(gltf, binchunk, idx):
    acc = gltf["accessors"][idx]
    view = gltf["bufferViews"][acc["bufferView"]]
    ncomp = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}[acc["type"]]
    dtype = {5126: np.float32, 5125: np.uint32, 5123: np.uint16}[acc["componentType"]]
    start = view.get("byteOffset", 0) + acc.get("byteOffset", 0)
    arr = np.frombuffer(binchunk, dtype, acc["count"] * ncomp, start)
    return arr.reshape(-1, ncomp) if ncomp > 1 else arr


def _mesh(gltf, binchunk, name):
    node = next(n for n in gltf["nodes"] if n.get("name") == name)
    prim = gltf["meshes"][node["mesh"]]["primitives"][0]
    pos = _accessor(gltf, binchunk, prim["attributes"]["POSITION"])
    idx = _accessor(gltf, binchunk, prim["indices"]).reshape(-1, 3)
    uv = (_accessor(gltf, binchunk, prim["attributes"]["TEXCOORD_0"])
          if "TEXCOORD_0" in prim["attributes"] else None)
    return pos, idx, uv, gltf["materials"][prim["material"]]


# ------------------------------------------------------------ package tests --

@needs_build
def test_manifest_valid_against_schema():
    import jsonschema
    schema = json.loads((SCRIPTS / "scene_manifest.schema.json").read_text())
    jsonschema.validate(json.loads((SCENE / "scene_manifest.json").read_text()), schema)


@needs_build
def test_stats_match_instances():
    m = json.loads((SCENE / "scene_manifest.json").read_text())
    assert m["stats"]["buildings"] == len(m["buildings"])
    assert m["stats"]["trees"] == len(m["trees"])
    assert m["stats"]["terrain_tiles"] == len(m["assets"]["terrain"]["tiles"])
    counts = {}
    for p in m["perimeter"]:
        counts[p["type"]] = counts.get(p["type"], 0) + 1
    assert counts == m["stats"]["perimeter_parts"]


@needs_build
def test_glb_nodes_match_manifest():
    m = json.loads((SCENE / "scene_manifest.json").read_text())
    g, _ = _read_glb(SCENE / "terrain.glb")
    assert {n["name"] for n in g["nodes"] if "mesh" in n} == \
        {t["name"] for t in m["assets"]["terrain"]["tiles"]}
    g, _ = _read_glb(SCENE / "buildings.glb")
    assert {n["name"] for n in g["nodes"] if "mesh" in n} == set(m["assets"]["buildings"]["nodes"])


@needs_build
def test_terrain_texture_orientation_and_up_normals():
    """North edge must sample the TOP of the photo (glTF v=0), east edge u=1,
    and triangles must face up - the classic silent errors in an engine port."""
    g, b = _read_glb(SCENE / "terrain.glb")
    pos, idx, uv, mat = _mesh(g, b, "terrain_0_0")
    north = pos[:, 2] <= pos[:, 2].min() + 1e-3          # glTF Z = -North
    east = pos[:, 0] >= pos[:, 0].max() - 1e-3
    assert np.allclose(uv[north, 1], 0.0, atol=1e-3), "north edge should be image top"
    assert np.allclose(uv[east, 0], 1.0, atol=1e-3), "east edge should be image right"
    a, bb, c = pos[idx[:, 0]], pos[idx[:, 1]], pos[idx[:, 2]]
    normals = np.cross(bb - a, c - a)                     # glTF front face = CCW
    assert (normals[:, 1] > 0).mean() > 0.99, "terrain triangles must face +Y (up)"
    assert "baseColorTexture" in mat["pbrMetallicRoughness"]


@needs_build
def test_walls_double_sided_and_roofs_textured():
    g, _ = _read_glb(SCENE / "buildings.glb")
    mats = {m["name"]: m for m in g["materials"]}
    assert "baseColorTexture" in mats["roof"]["pbrMetallicRoughness"]
    walls = [m for n, m in mats.items() if n.startswith("wall_")]
    assert walls and all(m.get("doubleSided") for m in walls)


@needs_build
def test_actors_identical_in_gazebo_and_manifest():
    """Gazebo and every other engine must put each person at the same place at
    the same sim time - otherwise camera labels and LiDAR disagree."""
    m = json.loads((SCENE / "scene_manifest.json").read_text())
    sdf = (SCENE.parent / f"{m['site']}.sdf").read_text()
    for actor in m["actors"]:
        block = re.search(rf'<actor name="{actor["name"]}">(.*?)</actor>', sdf, re.S).group(1)
        gz = re.findall(r"<time>([-\d.]+)</time><pose>([-\d. ]+)</pose>", block)
        assert len(gz) == len(actor["waypoints"])
        for (t, pose), w in zip(gz, actor["waypoints"]):
            x, y, z, _, _, yaw = map(float, pose.split())
            assert abs(float(t) - w["t"]) < 1e-3
            assert abs(x - w["x"]) < 1e-3 and abs(y - w["y"]) < 1e-3
            assert abs(z - (w["z_ground"] + 1.0)) < 1e-3     # Gazebo mesh root = hip
            assert abs(yaw - w["yaw"]) < 1e-4


@needs_build
def test_launch_pad_is_on_the_ground():
    """World origin is the launch pad, so terrain height there must be ~0."""
    g, b = _read_glb(SCENE / "terrain.glb")
    for node in g["nodes"]:
        if "mesh" not in node:
            continue
        pos, *_ = _mesh(g, b, node["name"])
        near = np.hypot(pos[:, 0], pos[:, 2]) < 1.0
        if near.any():
            assert np.abs(pos[near, 1]).max() < 0.5
            return
    pytest.fail("no terrain vertex near the origin")
