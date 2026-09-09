#!/usr/bin/env python3
"""
Settle whether the LiDAR header stamp marks the start or the end of a scan.

Why the obvious test does not work
----------------------------------
An earlier version of this script accumulated scans through a known trajectory
and measured map sharpness. That cannot work: shifting every scan's timeline by
the same 100 ms places scan k where scan k-1 belonged, but it shifts *all* scans
equally, so the map stays internally consistent and only second-order
velocity-variation effects show up. The two sequences it was tried on disagreed,
which is the expected outcome for a metric with no signal.

A uniform time shift is unobservable from LiDAR self-consistency alone. It is
only observable against another clock -- so this compares LiDAR-derived rotation
against the IMU gyroscope.

Method
------
1. For each scan build a 360-bin azimuth/range signature: the median range of
   points in a horizontal band, per one-degree azimuth bin. This is a compact
   rotational fingerprint of the surroundings.
2. Circular cross-correlation of consecutive signatures gives the inter-scan
   yaw increment, i.e. a LiDAR-derived yaw rate at 10 Hz. (Static structure
   appears to rotate opposite to the vehicle, hence the sign flip.)
3. Cross-correlate that against the gyro yaw rate over a range of lags. The lag
   that maximises correlation is the offset between the LiDAR timeline and the
   IMU timeline.

A lag near 0 means the stamp is the scan START and points span [stamp, stamp+T].
A lag near +T (one scan period) means it is the scan END.
"""
from __future__ import annotations

import argparse
import sqlite3
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import cdr  # noqa: E402

NBINS = 360


def scan_signature(pts, zmin, zmax, rmin, rmax):
    """Median range per azimuth bin, in the sensor frame."""
    xyz = np.stack([pts["x"], pts["y"], pts["z"]], 1).astype(np.float64)
    good = np.isfinite(xyz).all(1)
    xyz = xyz[good]
    if len(xyz) == 0:
        return None
    r = np.linalg.norm(xyz[:, :2], axis=1)
    keep = (r > rmin) & (r < rmax) & (xyz[:, 2] > zmin) & (xyz[:, 2] < zmax)
    if keep.sum() < 200:
        return None
    xyz, r = xyz[keep], r[keep]
    az = np.arctan2(xyz[:, 1], xyz[:, 0])
    b = ((az + np.pi) / (2 * np.pi) * NBINS).astype(int) % NBINS
    sig = np.full(NBINS, np.nan)
    order = np.argsort(b)
    b_s, r_s = b[order], r[order]
    edges = np.searchsorted(b_s, np.arange(NBINS + 1))
    for i in range(NBINS):
        lo, hi = edges[i], edges[i + 1]
        if hi > lo:
            sig[i] = np.median(r_s[lo:hi])
    return sig


def circular_yaw_shift(a, b, max_bins=40):
    """Bins to rotate `a` by to best match `b`, refined to sub-bin by a parabola."""
    m = ~np.isnan(a) & ~np.isnan(b)
    if m.sum() < NBINS // 3:
        return None
    a = np.where(np.isnan(a), np.nanmean(a), a)
    b = np.where(np.isnan(b), np.nanmean(b), b)
    a = a - a.mean()
    b = b - b.mean()
    shifts = np.arange(-max_bins, max_bins + 1)
    scores = np.array([np.dot(np.roll(a, s), b) for s in shifts])
    k = int(np.argmax(scores))
    if 0 < k < len(scores) - 1:
        y0, y1, y2 = scores[k - 1], scores[k], scores[k + 1]
        denom = y0 - 2 * y1 + y2
        delta = 0.5 * (y0 - y2) / denom if denom != 0 else 0.0
    else:
        delta = 0.0
    return (shifts[k] + delta) * (2 * np.pi / NBINS)


def load(con, name, decoder):
    row = con.execute("SELECT id FROM topics WHERE name=?", (name,)).fetchone()
    if row is None:
        return []
    return [decoder(bytes(d)) for (d,) in con.execute(
        "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (row[0],))]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("db3")
    ap.add_argument("--topic", default="/ouster0")
    ap.add_argument("--period", type=float, default=0.1)
    ap.add_argument("--scans", type=int, default=900, help="scans to analyse")
    ap.add_argument("--zmin", type=float, default=-0.6)
    ap.add_argument("--zmax", type=float, default=2.0)
    ap.add_argument("--rmin", type=float, default=3.0)
    ap.add_argument("--rmax", type=float, default=35.0)
    args = ap.parse_args()

    con = sqlite3.connect(f"file:{args.db3}?mode=ro", uri=True)

    imu = load(con, "/imu", cdr.decode_imu)
    it = np.array([m["t"] for m in imu])
    igz = np.array([m["gyro"][2] for m in imu])

    tid = con.execute("SELECT id FROM topics WHERE name=?", (args.topic,)).fetchone()[0]
    rows = con.execute(
        "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)).fetchall()

    # Choose the most-rotating stretch: that is where yaw rate has the most structure.
    win = int(args.scans * args.period * 100)
    if win < len(igz):
        csum = np.convolve(np.abs(igz), np.ones(win) / win, mode="valid")
        t_start = it[int(np.argmax(csum))]
    else:
        t_start = it[0]

    sigs, stamps = [], []
    for (blob,) in rows:
        m = cdr.decode_pointcloud2(bytes(blob))
        if m["t"] < t_start:
            continue
        s = scan_signature(m["points"], args.zmin, args.zmax, args.rmin, args.rmax)
        if s is not None:
            sigs.append(s)
            stamps.append(m["t"])
        if len(sigs) >= args.scans:
            break
    con.close()

    stamps = np.array(stamps)
    print(f"{len(sigs)} scans from t={stamps[0]:.2f} "
          f"({stamps[-1] - stamps[0]:.1f}s of data)")

    yaw_rate, mid_t = [], []
    for i in range(len(sigs) - 1):
        dt = stamps[i + 1] - stamps[i]
        if not (0.5 * args.period < dt < 2 * args.period):
            continue
        d = circular_yaw_shift(sigs[i], sigs[i + 1])
        if d is None:
            continue
        # Static structure rotates opposite to the vehicle.
        yaw_rate.append(-d / dt)
        mid_t.append(0.5 * (stamps[i] + stamps[i + 1]))
    yaw_rate = np.array(yaw_rate)
    mid_t = np.array(mid_t)
    print(f"{len(yaw_rate)} inter-scan yaw estimates, "
          f"|rate| max = {np.abs(yaw_rate).max():.3f} rad/s")

    lags = np.arange(-0.20, 0.2001, 0.005)
    best = (-2.0, 0.0)
    scores = []
    for lag in lags:
        g = np.interp(mid_t + lag, it, igz)
        if np.std(g) < 1e-9 or np.std(yaw_rate) < 1e-9:
            scores.append(0.0)
            continue
        c = float(np.corrcoef(yaw_rate, g)[0, 1])
        scores.append(c)
        if c > best[0]:
            best = (c, lag)
    scores = np.array(scores)
    corr, lag = best

    print(f"\nbest correlation {corr:+.4f} at lag {lag * 1000:+.0f} ms")
    print("  lag(ms)  corr")
    for l, s in zip(lags, scores):
        if abs(l - lag) < 0.051:
            mark = "  <==" if abs(l - lag) < 1e-9 else ""
            print(f"  {l * 1000:+7.0f}  {s:+.4f}{mark}")

    # The scan's mean observation time is stamp + T/2 if the stamp is the start,
    # stamp - T/2 if it is the end. We correlated using the raw stamp midpoints,
    # so a positive required lag of ~T/2 means start, ~-T/2 means end.
    half = args.period / 2
    d_start, d_end = abs(lag - half), abs(lag + half)
    verdict = "START" if d_start < d_end else "END"
    print(f"\nexpected lag if stamp = scan start: {half * 1000:+.0f} ms")
    print(f"expected lag if stamp = scan end:   {-half * 1000:+.0f} ms")
    print(f"=> stamp marks scan {verdict}")
    if corr < 0.5:
        print("   WARNING: weak correlation, treat as inconclusive.")
    if abs(d_start - d_end) < 0.02:
        print("   WARNING: the two hypotheses are nearly equidistant.")


if __name__ == "__main__":
    main()
