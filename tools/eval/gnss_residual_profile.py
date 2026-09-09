#!/usr/bin/env python3
"""
Where, in time, does a trajectory sit relative to the raw GNSS track?

`eval/validate.py` reports one aggregate GNSS residual per sequence. That is
enough to say a result is bad and useless for saying *why*, because a failure
confined to the last 10% of a run averages down into something that merely looks
mediocre. This prints the median residual per decile of time instead, which is
what localised the loop-closure failure during the in-pipeline GNSS work:

    two-stage          med per decile: 0.07 0.16 0.84 0.38 0.07 0.07 0.10 0.08 0.11 0.07
    in-pipeline        med per decile: 0.03 0.06 0.18 0.13 0.11 0.11 0.08 0.16 0.42 2.14

Read that way it is obvious the in-pipeline result tracked GNSS *better* than the
two-stage for 80% of the run and then fell apart exactly where the ICP loop
factors fired -- a conclusion the single rmse numbers (1.35 vs 1.10) actively hid.

The lever arm is compensated through each pose's own attitude, so the residual is
directly comparable to the receiver's quoted sigma (~0.5 m) rather than carrying a
constant 1.09 m antenna offset the way a naive comparison would.

Usage:
    python3 eval/gnss_residual_profile.py <label>=<sequence>=<trajectory.tum> ...
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "preproc"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "anchor"))
from gnss_posegraph import LEVER_ARM, load_gps, quat_to_R  # noqa: E402


def profile(label: str, seq: str, path: str, data: str = "/data", deciles: int = 10):
    gps = load_gps(f"{data}/{seq}-Charlie8/data.db3")
    if gps is None:
        print(f"{label:24s} {seq}: no valid GNSS in this sequence")
        return
    d = np.loadtxt(path)
    if d.ndim == 1:
        d = d[None, :]
    t, xyz, quat = d[:, 0], d[:, 1:4], d[:, 4:8]
    keep = (t >= gps["t"][0]) & (t <= gps["t"][-1])
    if keep.sum() < deciles:
        print(f"{label:24s} {seq}: only {keep.sum()} poses overlap the GNSS span")
        return
    t, xyz, quat = t[keep], xyz[keep], quat[keep]

    enu = np.stack([np.interp(t, gps["t"], gps["enu"][:, k]) for k in range(3)], axis=1)
    antenna = xyz + np.array([quat_to_R(q) @ LEVER_ARM for q in quat])
    r = np.linalg.norm(antenna[:, :2] - enu[:, :2], axis=1)

    med = " ".join(f"{np.median(c):.2f}" for c in np.array_split(r, deciles))
    print(f"{label:24s} {seq:30s} rmse {np.sqrt((r ** 2).mean()):.3f}  "
          f"med {np.median(r):.3f}  p95 {np.percentile(r, 95):.3f}  max {r.max():.3f}")
    print(f"{'':24s} {'median per decile of time:':30s} {med}")


def main():
    args = sys.argv[1:]
    if not args:
        raise SystemExit(__doc__)
    for arg in args:
        label, seq, path = arg.split("=", 2)
        profile(label, seq, path)


if __name__ == "__main__":
    main()
