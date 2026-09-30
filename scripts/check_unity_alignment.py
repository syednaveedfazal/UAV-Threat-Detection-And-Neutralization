#!/usr/bin/env python3
"""Measure how well Unity's scene lines up with the real aerial photo.

Input: the top-down capture written by Unity (Guardian > Capture Top-Down, or
GuardianMenu.BatchCapture) - unity_topdown_<region>.png + .json with its world
bounds. Reference: the aerial photo itself (the scene package's terrain
textures), cut to the same bounds and resolution.

Both images are reduced to edge strength first, so Unity's lighting and
shadows matter less than where things are. Then:

  * phase correlation gives the shift between them -> metres
  * the same score for mirrored / rotated copies of the Unity image shows
    whether an axis convention is wrong (the identity must win clearly)

Pass: identity wins and the shift is under --max-shift-m (default 0.5 m).

Usage:
    ~/.venvs/geo/bin/python scripts/check_unity_alignment.py \
        ~/UAV/data/bonn_poppelsdorf/world/scene/unity_topdown_compound.png
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage
from skimage.registration import phase_cross_correlation

Image.MAX_IMAGE_PIXELS = None


def reference_image(scene: Path, wmin, wmax, px: int) -> np.ndarray:
    """Aerial photo covering [wmin, wmax] (world ENU) at px x px, north up."""
    m = json.loads((scene / "scene_manifest.json").read_text())
    size = wmax[0] - wmin[0]
    mpp = size / px
    canvas = Image.new("RGB", (px, px))
    for t in m["assets"]["terrain"]["tiles"]:
        tex = Image.open(scene / "textures" / f"{t['name']}.jpg")
        tpx = int(round(t["size_m"] / mpp))
        left = int(round((t["min"][0] - wmin[0]) / mpp))
        top = int(round((wmax[1] - (t["min"][1] + t["size_m"])) / mpp))
        if left > px or top > px or left + tpx < 0 or top + tpx < 0:
            continue
        canvas.paste(tex.resize((tpx, tpx), Image.BILINEAR), (left, top))
    return np.asarray(canvas.convert("L"), np.float32)


def edges(img: np.ndarray) -> np.ndarray:
    g = ndimage.gaussian_filter(img, 1.5)
    e = np.hypot(ndimage.sobel(g, 0), ndimage.sobel(g, 1))
    return (e - e.mean()) / (e.std() + 1e-6)


def ncc(a: np.ndarray, b: np.ndarray) -> float:
    return float((a * b).mean())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("capture", type=Path)
    ap.add_argument("--max-shift-m", type=float, default=0.5)
    args = ap.parse_args()

    meta = json.loads(args.capture.with_suffix(".json").read_text())
    px, mpp = meta["px"], meta["m_per_px"]
    unity = np.asarray(Image.open(args.capture).convert("L"), np.float32)
    ref = reference_image(args.capture.parent, meta["world_min"], meta["world_max"], px)

    eu, er = edges(unity), edges(ref)
    shift, _, _ = phase_cross_correlation(er, eu, upsample_factor=10, normalization=None)
    dy, dx = shift
    aligned = ndimage.shift(eu, shift, order=1)
    candidates = {
        "identity": ncc(er, aligned),
        "mirror east-west": ncc(er, eu[:, ::-1]),
        "mirror north-south": ncc(er, eu[::-1, :]),
        "rotated 180": ncc(er, eu[::-1, ::-1]),
        "rotated 90": ncc(er, np.rot90(eu)),
    }
    best = max(candidates, key=candidates.get)
    shift_m = float(np.hypot(dx, dy) * mpp)

    print(f"capture   {args.capture.name}: {px} px, {mpp * 100:.1f} cm/px")
    # phase_cross_correlation returns the shift that moves Unity onto the photo,
    # so Unity's displacement is its negative (image rows grow southwards).
    print(f"offset    Unity is displaced east {-dx * mpp:+.3f} m, north {dy * mpp:+.3f} m "
          f"from the photo  (|offset| {shift_m:.3f} m)")
    for k, v in sorted(candidates.items(), key=lambda kv: -kv[1]):
        print(f"  match   {k:18s} {v:.3f}")
    ok = best == "identity" and shift_m <= args.max_shift_m
    margin = candidates["identity"] - max(v for k, v in candidates.items() if k != "identity")
    print(f"result    {'PASS' if ok else 'FAIL'}: best={best}, margin over next {margin:.3f}, "
          f"limit {args.max_shift_m} m")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
