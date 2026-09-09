#!/usr/bin/env python3
"""
Validate a generated ground truth trajectory.

Four checks, in increasing order of how much a reviewer will care:

1. GNSS residual -- should sit at or below the receiver's own ~0.5 m sigma.
2. Loop closure error -- only meaningful on the four sequences that return to
   their start (ditches, featuresAndGps, insideGarage, niceFeatures).
3. Agreement with an independent front-end (KISS-ICP, or a second better_fastlio2
   run with different settings). Their disagreement is the uncertainty estimate
   you can actually quote.
4. Agreement with the sponsor's /pose. You cannot publish their trajectory, but
   nothing stops you computing against it privately and stating the number --
   "agrees with an independent commercial INS solution to X cm RMSE" is one
   sentence in the paper and pre-empts the obvious question.

Alignment is Umeyama without scale (SE(3)), matching evo's `--align` convention,
so numbers here are comparable to evo output.
"""
from __future__ import annotations

import argparse
import sqlite3
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "preproc"))
import cdr  # noqa: E402


def read_tum(path):
    d = np.loadtxt(path)
    if d.ndim == 1:
        d = d[None, :]
    return d[:, 0], d[:, 1:4]


def umeyama(src, dst):
    """SE(3) carrying src onto dst, no scale."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    H = (src - mu_s).T @ (dst - mu_d)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    return R, mu_d - R @ mu_s


def associate(ta, pa, tb, pb, max_dt=0.02):
    """Nearest-time association."""
    idx = np.searchsorted(tb, ta)
    idx = np.clip(idx, 1, len(tb) - 1)
    pick = np.where(np.abs(tb[idx - 1] - ta) < np.abs(tb[idx] - ta), idx - 1, idx)
    ok = np.abs(tb[pick] - ta) < max_dt
    return pa[ok], pb[pick[ok]], ok.sum()


def ate(pa, pb):
    R, t = umeyama(pa, pb)
    err = np.linalg.norm((R @ pa.T).T + t - pb, axis=1)
    return err


def ate_split(pa, pb):
    """3D, horizontal-only and vertical-only error after one common alignment.

    Worth separating because the vertical channel fails independently: an INS with
    no GNSS aiding drifts in z far faster than in x/y, so a 3D number can be
    dominated by a vertical disagreement that says nothing about the horizontal
    trajectory. insideGarage is exactly that case."""
    R, t = umeyama(pa, pb)
    d = (R @ pa.T).T + t - pb
    return (np.linalg.norm(d, axis=1),
            np.linalg.norm(d[:, :2], axis=1),
            np.abs(d[:, 2]))


def rpe(ta, pa, pb, delta=10.0):
    """Relative translation error over `delta`-second windows -- the metric that
    actually matters for evaluating odometry on this dataset."""
    out = []
    j = 0
    for i in range(len(ta)):
        while j < len(ta) and ta[j] - ta[i] < delta:
            j += 1
        if j >= len(ta):
            break
        da = np.linalg.norm(pa[j] - pa[i])
        db = np.linalg.norm(pb[j] - pb[i])
        out.append(abs(da - db))
    return np.array(out)


def load_sponsor_pose(db3):
    con = sqlite3.connect(f"file:{db3}?mode=ro", uri=True)
    row = con.execute("SELECT id FROM topics WHERE name='/pose'").fetchone()
    if row is None:
        con.close()
        return None, None
    P = [cdr.decode_pose(bytes(d)) for (d,) in con.execute(
        "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (row[0],))]
    con.close()
    return np.array([p["t"] for p in P]), np.array([p["pos"] for p in P])


def load_gps_enu(db3):
    con = sqlite3.connect(f"file:{db3}?mode=ro", uri=True)
    row = con.execute("SELECT id FROM topics WHERE name='/gps'").fetchone()
    if row is None:
        con.close()
        return None, None
    G = [cdr.decode_gps(bytes(d)) for (d,) in con.execute(
        "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (row[0],))]
    con.close()
    G = [g for g in G if g["status"] != 0]
    if not G:
        return None, None
    o = G[0]
    return (np.array([g["t"] for g in G]),
            np.array([[g["easting"] - o["easting"], g["northing"] - o["northing"],
                       g["alt"] - o["alt"]] for g in G]))


def report(name, err, unit="m"):
    if len(err) == 0:
        print(f"  {name:34s} no overlap")
        return
    print(f"  {name:34s} rmse {np.sqrt((err ** 2).mean()):6.3f}  "
          f"med {np.median(err):6.3f}  p95 {np.percentile(err, 95):6.3f}  "
          f"max {err.max():6.3f} {unit}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("traj", help="generated ground truth, TUM")
    ap.add_argument("--db3", help="source data.db3 for /gps and /pose")
    ap.add_argument("--compare", help="independent trajectory, TUM (e.g. KISS-ICP)")
    ap.add_argument("--rpe-delta", type=float, default=10.0)
    ap.add_argument("--loop", action="store_true",
                    help="report start-to-end distance (only for looping sequences)")
    args = ap.parse_args()

    t, p = read_tum(args.traj)
    path_len = np.linalg.norm(np.diff(p, axis=0), axis=1).sum()
    print(f"{Path(args.traj).name}: {len(t)} poses, {t[-1] - t[0]:.1f} s, "
          f"{path_len:.1f} m")

    if args.loop:
        print(f"  {'loop closure (start to end)':34s} "
              f"{np.linalg.norm(p[0] - p[-1]):.3f} m over {path_len:.1f} m")

    if args.db3:
        gt, gp = load_gps_enu(args.db3)
        if gt is not None:
            a, b, n = associate(t, p, gt, gp, max_dt=0.05)
            if n:
                # Already in ENU after anchoring, so compare directly rather than
                # re-aligning -- re-aligning would hide a global offset.
                d = np.linalg.norm(a[:, :2] - b[:, :2], axis=1)
                report(f"vs GNSS ({n} matched, no align)", d)
                report("vs GNSS (aligned)", ate(a, b))
        else:
            print(f"  {'vs GNSS':34s} no valid fixes (expected for insideGarage)")

        st, sp = load_sponsor_pose(args.db3)
        if st is not None:
            a, b, n = associate(t, p, st, sp, max_dt=0.05)
            if n:
                e3, eh, ev = ate_split(a, b)
                report(f"vs sponsor /pose ({n} matched)", e3)
                report("  ... horizontal only", eh)
                report("  ... vertical only", ev)
                r = rpe(t[:len(a)], a, b, args.rpe_delta)
                report(f"vs sponsor RPE @ {args.rpe_delta:.0f}s", r)
                print("      (private cross-check -- do not publish the sponsor "
                      "trajectory, only this number)")

    if args.compare:
        ct, cp = read_tum(args.compare)
        a, b, n = associate(t, p, ct, cp, max_dt=0.05)
        if n:
            report(f"vs {Path(args.compare).name} ({n} matched)", ate(a, b))
            report(f"vs {Path(args.compare).name} RPE @ {args.rpe_delta:.0f}s",
                   rpe(t[:len(a)], a, b, args.rpe_delta))


if __name__ == "__main__":
    main()
