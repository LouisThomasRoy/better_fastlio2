#!/usr/bin/env python3
"""
Convert better_fastlio2's transformations.pcd into a TUM trajectory.

better_fastlio2 does not write a TUM file. It offers two outputs:

  * `rosservice call /save_pose` -> optimized_pose.txt, a plain rotation+
    translation dump with NO timestamps, which is useless for evaluation.
  * `rosservice call /save_map`  -> transformations.pcd, the cloudKeyPoses6D
    cloud, whose point type (PointXYZIRPYTRGB in include/common_lib.h) carries
    x, y, z, roll, pitch, yaw AND a double `time`.

Only the second is usable, so this reads it. Note that saveMap() is never called
automatically -- the call at laserMapping.cpp:2425 is commented out -- so the
service must be invoked before the node exits.

Roll/pitch/yaw are converted with Rz(yaw) Ry(pitch) Rx(roll), matching the repo's
Exp(roll, pitch, yaw) helper.

With --enu, the trajectory is additionally moved from better_fastlio2's world
frame (the first IMU body frame: arbitrary yaw, arbitrary origin) into the local
ENU frame, using the map_from_enu.txt our addGPSFactor() writes alongside the
pcd. That is a rigid change of coordinates and nothing more -- the GNSS factors
already ran inside the node's own iSAM2 graph, so there is no second optimiser
here. Because the transform is yaw-only, applying its inverse is exactly
`p -> Rz(-yaw)(p - t)` and `yaw_i -> yaw_i - yaw`, with roll and pitch untouched.
"""
from __future__ import annotations

import argparse
import json
import sqlite3
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "preproc"))
import cdr  # noqa: E402

PCD_TYPE = {("F", 4): "<f4", ("F", 8): "<f8",
            ("U", 1): "<u1", ("U", 2): "<u2", ("U", 4): "<u4", ("U", 8): "<u8",
            ("I", 1): "<i1", ("I", 2): "<i2", ("I", 4): "<i4", ("I", 8): "<i8"}


def read_pcd(path: Path):
    """Read a binary or ASCII PCD into a structured array.

    PCL's binary writer memcpy's the raw point structs, padding included, and
    compensates by emitting `_` filler entries in the FIELDS line. Building the
    dtype straight from the header therefore reproduces the in-memory layout.
    """
    with open(path, "rb") as fh:
        header, hdr_bytes = {}, 0
        while True:
            line = fh.readline()
            if not line:
                raise ValueError(f"{path}: no DATA line, truncated header?")
            hdr_bytes += len(line)
            text = line.decode("ascii", "replace").strip()
            if not text or text.startswith("#"):
                continue
            key, _, val = text.partition(" ")
            header[key.upper()] = val.strip()
            if key.upper() == "DATA":
                break
        payload = fh.read()

    fields = header["FIELDS"].split()
    sizes = [int(v) for v in header["SIZE"].split()]
    types = [v.upper() for v in header["TYPE"].split()]
    counts = ([int(v) for v in header["COUNT"].split()]
              if "COUNT" in header else [1] * len(fields))
    npoints = int(header["POINTS"]) if "POINTS" in header else (
        int(header["WIDTH"]) * int(header["HEIGHT"]))

    names, formats, offsets, off, pad = [], [], [], 0, 0
    for f, s, t, c in zip(fields, sizes, types, counts):
        key = (t, s)
        if key not in PCD_TYPE:
            raise ValueError(f"{path}: unsupported field type {t}{s}")
        for k in range(c):
            nm = f if c == 1 else f"{f}_{k}"
            if nm == "_" or nm.startswith("_"):
                nm = f"__pad{pad}"
                pad += 1
            names.append(nm)
            formats.append(PCD_TYPE[key])
            offsets.append(off)
            off += s

    dt = np.dtype({"names": names, "formats": formats,
                   "offsets": offsets, "itemsize": off})

    fmt = header["DATA"].lower()
    if fmt == "ascii":
        rows = [ln.split() for ln in payload.decode().splitlines() if ln.strip()]
        arr = np.zeros(len(rows), dtype=dt)
        real = [n for n in names if not n.startswith("__pad")]
        for i, row in enumerate(rows):
            for n, v in zip(real, row):
                arr[n][i] = float(v)
        return arr, header
    if fmt != "binary":
        raise ValueError(f"{path}: DATA '{fmt}' not supported "
                         "(binary_compressed needs LZF decoding)")

    need = npoints * dt.itemsize
    if len(payload) < need:
        raise ValueError(
            f"{path}: expected {need} bytes for {npoints} points of stride "
            f"{dt.itemsize}, got {len(payload)}.\n"
            f"  FIELDS {header['FIELDS']}\n  SIZE {header['SIZE']}\n"
            f"  TYPE {header['TYPE']}\n"
            "  The header does not describe the on-disk layout; please report "
            "these lines so the reader can be corrected.")
    return np.frombuffer(payload[:need], dtype=dt), header


def rpy_to_quat(roll, pitch, yaw):
    """Rz(yaw) Ry(pitch) Rx(roll) -> quaternion [x, y, z, w]."""
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
    return np.stack([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ], axis=1)


def read_map_from_enu(path: Path):
    """Parse the node's map_from_enu.txt -> (yaw, t) or None if it never aligned."""
    vals = {}
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        key, _, rest = line.partition(" ")
        vals[key] = rest.split()
    if not vals.get("aligned") or int(vals["aligned"][0]) != 1:
        return None
    return float(vals["yaw"][0]), np.array([float(v) for v in vals["t"]]), vals


def gnss_anchor(db3: str):
    """lat/lon/alt of the first valid fix -- the origin the node's ENU is relative
    to. The node never sees it (it subscribes to the already-local /gps/odom_enu),
    so it is recovered here from the source bag, the same way the offline stage
    did it."""
    con = sqlite3.connect(f"file:{db3}?mode=ro", uri=True)
    row = con.execute("SELECT id FROM topics WHERE name='/gps'").fetchone()
    if row is None:
        con.close()
        return None
    for (d,) in con.execute(
            "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (row[0],)):
        g = cdr.decode_gps(bytes(d))
        if g["status"] != 0:
            con.close()
            return {"lat": g["lat"], "lon": g["lon"], "alt": g["alt"],
                    "easting": g["easting"], "northing": g["northing"]}
    con.close()
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pcd", help="transformations.pcd from rosservice call /save_map")
    ap.add_argument("out", help="output .tum")
    ap.add_argument("--enu", default=None,
                    help="map_from_enu.txt written by the node's GNSS stage; "
                         "applies its inverse so the output is local ENU")
    ap.add_argument("--anchor-db3", default=None,
                    help="source data.db3, to record the ENU origin lat/lon in a "
                         "sidecar .json (only meaningful with --enu)")
    args = ap.parse_args()

    arr, header = read_pcd(Path(args.pcd))
    present = [n for n in arr.dtype.names if not n.startswith("__pad")]
    print(f"{args.pcd}: {len(arr)} keyframes, fields {present}")

    missing = [f for f in ("x", "y", "z", "roll", "pitch", "yaw", "time")
               if f not in arr.dtype.names]
    if missing:
        raise SystemExit(
            f"missing field(s) {missing}. This does not look like "
            "cloudKeyPoses6D -- transformations.pcd is the right file, "
            "trajectory.pcd only holds positions.")

    t = np.asarray(arr["time"], dtype=float)
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(float)
    quat = rpy_to_quat(np.asarray(arr["roll"], float),
                       np.asarray(arr["pitch"], float),
                       np.asarray(arr["yaw"], float))

    order = np.argsort(t)
    t, xyz, quat = t[order], xyz[order], quat[order]

    meta = {"frame": ("LiDAR-inertial frame, gravity-aligned, arbitrary yaw and "
                      "origin"),
            "keyframes": int(len(t)),
            "source": "better_fastlio2 iSAM2 keyframe poses (single optimisation)"}
    if args.enu:
        fit = read_map_from_enu(Path(args.enu))
        if fit is None:
            print(f"  {args.enu} reports aligned 0: the node added no GNSS factor, "
                  "leaving the output in the arbitrary LiDAR frame")
        else:
            yaw_a, t_a, vals = fit
            roll, pitch, yaw = (np.asarray(arr[k], float)[order]
                                for k in ("roll", "pitch", "yaw"))
            c, s_ = np.cos(-yaw_a), np.sin(-yaw_a)
            Rinv = np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])
            xyz = (Rinv @ (xyz - t_a).T).T
            quat = rpy_to_quat(roll, pitch, yaw - yaw_a)
            nfac = int(vals["factors"][0])
            print(f"  ENU: un-rotated by {np.degrees(yaw_a):+.3f} deg about z; "
                  f"the node used {nfac} GNSS factors "
                  f"(fit rms {float(vals['align_rms'][0]):.3f} m over "
                  f"{float(vals['align_path'][0]):.1f} m, "
                  f"{int(vals['rejected'][0])} fixes rejected)")
            meta["frame"] = "local ENU, x=east y=north z=up, poses are the IMU frame"
            meta["gnss_factors"] = nfac
            meta["gnss_rejected"] = int(vals["rejected"][0])
            meta["enu_to_map_yaw_deg"] = float(np.degrees(yaw_a))
            meta["enu_to_map_fit_rms_m"] = float(vals["align_rms"][0])
            meta["note"] = ("GNSS position factors were added inside "
                            "better_fastlio2's own iSAM2 graph; this file is that "
                            "graph's output under a rigid change of coordinates. "
                            "GPS is DGPS/SBAS (status 2, sigma ~0.5 m), not RTK: it "
                            "bounds drift, local accuracy comes from the "
                            "LiDAR-inertial front-end.")
            if args.anchor_db3:
                anchor = gnss_anchor(args.anchor_db3)
                if anchor:
                    meta["anchor"] = anchor

    path_len = np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum()
    print(f"  span {t[-1] - t[0]:.1f} s, path {path_len:.1f} m, "
          f"start-to-end {np.linalg.norm(xyz[0] - xyz[-1]):.2f} m")
    if t[0] < 1e8:
        print("  WARNING: timestamps look relative, not absolute epoch seconds. "
              "The GNSS anchoring stage needs absolute time to associate fixes.")

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(args.out, np.column_stack([t, xyz, quat]), fmt="%.9f")
    Path(args.out).with_suffix(".json").write_text(json.dumps(meta, indent=2))
    print(f"wrote {args.out} and {Path(args.out).with_suffix('.json')}")


if __name__ == "__main__":
    main()
