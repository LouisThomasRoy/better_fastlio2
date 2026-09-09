#!/usr/bin/env python3
"""Sanity-check a converted ROS 1 bag: point layout, de-skew times, ring range.

Run after any converter change. The checks correspond to the things that silently
break better_fastlio2 rather than making it fail loudly:
  * every point finite (inf coordinates produce NaN poses in ICP)
  * per-point `time` inside [0, scan period)
  * the LAST point's time > 0, which is preprocess.cpp's gate for trusting
    per-point times at all
  * ring within scan_line, since preprocess.cpp drops ring >= N_SCANS
  * bag timestamps non-decreasing, which the dual-lidar reorder buffer must
    preserve
"""
from __future__ import annotations

import argparse

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

TS = get_typestore(Stores.ROS1_NOETIC)
PT = np.dtype({"names": ["x", "y", "z", "intensity", "time", "ring"],
               "formats": ["<f4"] * 5 + ["<u2"],
               "offsets": [0, 4, 8, 12, 16, 20], "itemsize": 24})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--topic", default=None, help="lidar topic (default: autodetect)")
    ap.add_argument("--period", type=float, default=0.1)
    ap.add_argument("--scan-line", type=int, default=64)
    args = ap.parse_args()

    with Reader(args.bag) as r:
        print(f"{args.bag}: {r.duration / 1e9:.1f}s, {r.message_count} msgs")
        for c in r.connections:
            print(f"  {c.topic:18s} {c.msgtype:34s} n={c.msgcount}")

        topic = args.topic
        if topic is None:
            cands = [c.topic for c in r.connections
                     if c.msgtype == "sensor_msgs/msg/PointCloud2"]
            if not cands:
                raise SystemExit("no PointCloud2 topic")
            topic = cands[0]
        cons = [c for c in r.connections if c.topic == topic]

        prev = -1
        mono = True
        n_bad_finite = n_bad_time = n_bad_ring = n_bad_last = 0
        widths, l0, l1 = [], 0, 0
        nscan = 0
        for c, t, raw in r.messages(connections=cons):
            if t < prev:
                mono = False
            prev = t
            m = TS.deserialize_ros1(raw, c.msgtype)
            a = np.frombuffer(m.data.tobytes(), dtype=PT)
            nscan += 1
            widths.append(len(a))
            if len(a) == 0:
                continue
            xyz = np.stack([a["x"], a["y"], a["z"]], 1)
            if not np.isfinite(xyz).all():
                n_bad_finite += 1
            tt = a["time"]
            if tt.min() < -1e-6 or tt.max() >= args.period:
                n_bad_time += 1
            if tt[-1] <= 0:
                n_bad_last += 1
            if a["ring"].max() >= args.scan_line:
                n_bad_ring += 1
            l0 += int((a["ring"] < 32).sum())
            l1 += int((a["ring"] >= 32).sum())

        w = np.array(widths)
        print(f"\n  topic {topic}: {nscan} scans, "
              f"width min/med/max {w.min()}/{int(np.median(w))}/{w.max()}")
        if l1:
            tot = l0 + l1
            print(f"  ring split: {l0} below 32, {l1} at/above 32 "
                  f"({100.0 * l1 / tot:.1f}% second lidar)")
        checks = [
            ("all points finite", n_bad_finite == 0, f"{n_bad_finite} bad scans"),
            (f"time in [0,{args.period})", n_bad_time == 0, f"{n_bad_time} bad scans"),
            ("last point time > 0", n_bad_last == 0, f"{n_bad_last} bad scans"),
            (f"ring < scan_line({args.scan_line})", n_bad_ring == 0, f"{n_bad_ring} bad scans"),
            ("bag timestamps non-decreasing", mono, "out of order"),
        ]
        ok = True
        for name, passed, detail in checks:
            print(f"  [{'PASS' if passed else 'FAIL'}] {name}"
                  + ("" if passed else f"  -- {detail}"))
            ok &= passed
        raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
