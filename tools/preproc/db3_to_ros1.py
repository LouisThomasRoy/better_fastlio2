#!/usr/bin/env python3
"""
Convert a Charlie8 rosbag2 .db3 into a ROS 1 bag that better_fastlio2 can consume.

What this fixes, and why each fix is needed
-------------------------------------------
1. LiDAR points carry `rowIdx`/`colIdx` but no `ring` or per-point timestamp, so
   nothing can de-skew them. `rowIdx` (0..31) becomes `ring`; `colIdx` is
   monotonic over 0..1400 within a scan, so `time = colIdx / n_cols * period`.
   Emitted in the Velodyne layout FAST-LIO expects: named x, y, z, intensity,
   time (float32, seconds), ring (uint16).

2. 4-5% of points are non-finite (the driver emits inf for non-returns rather
   than dropping them). These are removed -- most ICP implementations turn them
   into NaN poses.

3. The recorded IMU `orientation` is unusable: rotating measured gravity by it
   gives e.g. [8.91, 3.36, 2.39] instead of [0, 0, 9.81], and it is not
   self-consistent in ditches. Replaced with a Madgwick estimate from the raw
   gyro/accel, which are healthy. Covariances are filled from the MTI-200
   datasheet instead of being left as zeros.

4. `/gps` is a custom prs_sensor_msgs/Gps. Emitted both as sensor_msgs/NavSatFix
   and as a local-ENU nav_msgs/Odometry built from the pre-computed UTM
   easting/northing, which is what the offline anchoring stage consumes.

Note on the stamp convention: better_fastlio2's Velodyne handler only trusts
per-point times when the last point's time is > 0, so times are always emitted
as non-negative offsets from the message stamp. If the recorded stamp turns out
to mark scan *end* rather than scan start, pass --stamp-is end and the stamp is
shifted back by one scan period so that invariant still holds. Use
`determine_stamp_convention.py` to settle which it is.
"""
from __future__ import annotations

import argparse
import sqlite3
import sys
from pathlib import Path

import heapq

import numpy as np
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore

sys.path.insert(0, str(Path(__file__).resolve().parent))
import cdr  # noqa: E402
import madgwick  # noqa: E402
from extrinsics import Extrinsics  # noqa: E402

TS = get_typestore(Stores.ROS1_NOETIC)

# XSENS MTI-200-2A8G4 datasheet noise densities, converted to per-sample
# variances at 100 Hz. The recorded covariances are all zero, which many
# estimators read as "unknown" and silently substitute defaults for.
GYRO_VAR = (0.007 * np.pi / 180.0) ** 2 * 100.0     # 0.007 deg/s/sqrt(Hz)
ACCEL_VAR = (60e-6 * 9.80665) ** 2 * 100.0          # 60 ug/sqrt(Hz)
ORIENT_VAR = np.deg2rad(1.0) ** 2                   # Madgwick roll/pitch, order 1 deg

LIDAR_PERIOD = 0.1     # 10 Hz

# Seconds added to each recorded lidar stamp so the emitted stamp marks the true
# start of the scan. Measured by correlating lidar-derived yaw rate against the
# gyro (determine_stamp_convention.py): the acquisition midpoint sits about
# 20 ms BEFORE the recorded stamp, so acquisition spans roughly
# [stamp-70ms, stamp+30ms] and the start is at stamp-70ms.
# The estimate is soft -- the correlation peak is broad (yaw rate is a smooth
# signal) and per-sequence estimates ranged 0 to -40 ms. Treat -0.07 as a
# starting point and confirm with the A/B run described in the README.
DEFAULT_TIME_SHIFT = -0.07
N_COLS = 1401          # colIdx spans 0..1400 in every scan checked

PC_DTYPE = np.dtype({
    "names":   ["x", "y", "z", "intensity", "time", "ring"],
    "formats": ["<f4", "<f4", "<f4", "<f4", "<f4", "<u2"],
    "offsets": [0, 4, 8, 12, 16, 20],
    "itemsize": 24,
})
PC_FIELDS = [("x", 0, 7), ("y", 4, 7), ("z", 8, 7),
             ("intensity", 12, 7), ("time", 16, 7), ("ring", 20, 4)]


def _time_msg(t: float):
    sec = int(np.floor(t))
    return TS.types["builtin_interfaces/msg/Time"](sec=sec,
                                                   nanosec=int(round((t - sec) * 1e9)))


def _header(t: float, frame: str, seq: int = 0):
    # ROS 1 headers carry a sequence counter that ROS 2 dropped.
    return TS.types["std_msgs/msg/Header"](seq=seq, stamp=_time_msg(t), frame_id=frame)


def make_imu(t, frame, quat_xyzw, gyro, acc, seq=0):
    q = TS.types["geometry_msgs/msg/Quaternion"](
        x=float(quat_xyzw[0]), y=float(quat_xyzw[1]),
        z=float(quat_xyzw[2]), w=float(quat_xyzw[3]))
    return TS.types["sensor_msgs/msg/Imu"](
        header=_header(t, frame, seq),
        orientation=q,
        orientation_covariance=np.diag([ORIENT_VAR, ORIENT_VAR, 4 * ORIENT_VAR]).ravel(),
        angular_velocity=TS.types["geometry_msgs/msg/Vector3"](
            x=float(gyro[0]), y=float(gyro[1]), z=float(gyro[2])),
        angular_velocity_covariance=np.diag([GYRO_VAR] * 3).ravel(),
        linear_acceleration=TS.types["geometry_msgs/msg/Vector3"](
            x=float(acc[0]), y=float(acc[1]), z=float(acc[2])),
        linear_acceleration_covariance=np.diag([ACCEL_VAR] * 3).ravel(),
    )


def make_pointcloud(t, frame, pts: np.ndarray, seq=0):
    fields = [
        TS.types["sensor_msgs/msg/PointField"](name=n, offset=o, datatype=d, count=1)
        for n, o, d in PC_FIELDS
    ]
    raw = pts.tobytes()
    return TS.types["sensor_msgs/msg/PointCloud2"](
        header=_header(t, frame, seq),
        height=1, width=len(pts), fields=fields, is_bigendian=False,
        point_step=PC_DTYPE.itemsize, row_step=PC_DTYPE.itemsize * len(pts),
        data=np.frombuffer(raw, dtype=np.uint8), is_dense=True,
    )


def make_navsatfix(t, frame, g, seq=0):
    # status: 0 = no fix in the recorded stream (insideGarage), 2 = DGPS/SBAS.
    # Map onto NavSatStatus: -1 NO_FIX, 0 FIX, 2 GBAS_FIX.
    status_val = -1 if g["status"] == 0 else 2
    status = TS.types["sensor_msgs/msg/NavSatStatus"](status=status_val, service=1)
    cov = np.diag([g["lon_err"] ** 2, g["lat_err"] ** 2,
                   (2.0 * max(g["lat_err"], g["lon_err"])) ** 2]).ravel()
    return TS.types["sensor_msgs/msg/NavSatFix"](
        header=_header(t, frame, seq), status=status,
        latitude=g["lat"], longitude=g["lon"], altitude=g["alt"],
        position_covariance=cov,
        position_covariance_type=2,   # DIAGONAL_KNOWN
    )


def make_gps_odom(t, frame, child, e, n, u, g, seq=0):
    pose_cov = np.zeros(36)
    pose_cov[0] = g["lon_err"] ** 2
    pose_cov[7] = g["lat_err"] ** 2
    pose_cov[14] = (2.0 * max(g["lat_err"], g["lon_err"])) ** 2
    pose_cov[21] = pose_cov[28] = pose_cov[35] = 1e6   # orientation unobserved
    pose = TS.types["geometry_msgs/msg/Pose"](
        position=TS.types["geometry_msgs/msg/Point"](x=float(e), y=float(n), z=float(u)),
        orientation=TS.types["geometry_msgs/msg/Quaternion"](x=0.0, y=0.0, z=0.0, w=1.0))
    twist = TS.types["geometry_msgs/msg/Twist"](
        linear=TS.types["geometry_msgs/msg/Vector3"](x=0.0, y=0.0, z=0.0),
        angular=TS.types["geometry_msgs/msg/Vector3"](x=0.0, y=0.0, z=0.0))
    return TS.types["nav_msgs/msg/Odometry"](
        header=_header(t, frame, seq), child_frame_id=child,
        pose=TS.types["geometry_msgs/msg/PoseWithCovariance"](pose=pose, covariance=pose_cov),
        twist=TS.types["geometry_msgs/msg/TwistWithCovariance"](
            twist=twist, covariance=np.full(36, 1e6)),
    )


def make_twist(t, frame, speed, turn_rate, seq=0):
    return TS.types["geometry_msgs/msg/TwistStamped"](
        header=_header(t, frame, seq),
        twist=TS.types["geometry_msgs/msg/Twist"](
            linear=TS.types["geometry_msgs/msg/Vector3"](x=0.0, y=float(speed), z=0.0),
            angular=TS.types["geometry_msgs/msg/Vector3"](x=0.0, y=0.0, z=float(turn_rate))),
    )


def convert_cloud(msg, time_shift: float, n_cols: int, period: float):
    """Source cloud -> (stamp, Velodyne-layout points). Returns None if empty."""
    pts = msg["points"]
    if len(pts) == 0:
        return None
    xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=1).astype(np.float32)
    good = np.isfinite(xyz).all(axis=1)
    if not good.any():
        return None
    pts = pts[good]
    xyz = xyz[good]

    out = np.empty(len(pts), dtype=PC_DTYPE)
    out["x"], out["y"], out["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    out["intensity"] = pts["intensity"].astype(np.float32)
    out["ring"] = pts["rowIdx"].astype(np.uint16)
    out["time"] = (pts["colIdx"].astype(np.float32) / float(n_cols)) * period

    stamp = msg["t"] + time_shift
    return stamp, out




# --------------------------------------------------------------------------
# Dual-LiDAR merging
# --------------------------------------------------------------------------
# better_fastlio2 subscribes to exactly one lid_topic, so using both OS1-32s means
# merging them into a single cloud before it ever sees them. Three things make that
# more than a concatenation:
#
# 1. The two units are NOT synchronised. Measured inter-lidar header stamp offsets
#    run 8-47 ms, so at 2-3 m/s a naive concatenation smears the second cloud by up
#    to 11 cm. Points are therefore placed on one absolute timeline and the merged
#    scan is defined by lidar0's 100 ms window; lidar1 points are pulled from
#    whichever of its scans overlap that window, which may be two.
#
# 2. Ring indices collide -- both report 0..31. lidar1 is offset to 32..63 and the
#    config sets scan_line: 64, because preprocess.cpp drops any point with
#    ring >= N_SCANS.
#
# 3. `blind` is a single radius measured from the cloud origin, which cannot express
#    "too close to sensor A" and "too close to sensor B" once the clouds share a
#    frame. Self-return rejection is therefore done here, per lidar, in each
#    sensor's own frame, before the transform into the body frame.
#
# Both clouds are expressed in the IMU/body frame, so the dual config sets
# extrinsic_T/R to identity.


class LidarMerger:
    """Pairs lidar0 and lidar1 scans onto lidar0's window.

    lidar1 scans are buffered because the one covering the end of a lidar0 window
    can arrive after it in stamp order. A lidar0 window is released once buffered
    lidar1 data extends past its end, which bounds the delay to a couple of scans.
    """

    RING_OFFSET = 32

    def __init__(self, extrinsics: Extrinsics, primary: str, secondary: str,
                 period: float, n_cols: int, blind: float):
        self.period = period
        self.n_cols = n_cols
        self.blind = blind
        self.primary = primary
        self.secondary = secondary
        self.T = {
            primary: extrinsics.homogeneous("lidar0"),
            secondary: extrinsics.homogeneous("lidar1"),
        }
        self.pending = []      # released-in-order lidar0 windows
        self.buf = []          # (t_start, t_end, xyz, intensity, ring, t_abs)
        self.stats = {"merged": 0, "sec_points": 0, "pri_points": 0, "dropped_blind": 0}

    def _prepare(self, msg, time_shift, frame, ring_offset):
        """Non-finite + blind filter in the sensor frame, then transform to body."""
        pts = msg["points"]
        if len(pts) == 0:
            return None
        xyz = np.stack([pts["x"], pts["y"], pts["z"]], 1).astype(np.float64)
        good = np.isfinite(xyz).all(1)
        if not good.any():
            return None
        xyz = xyz[good]
        pts = pts[good]
        rng = np.linalg.norm(xyz, axis=1)
        keep = rng > self.blind
        self.stats["dropped_blind"] += int((~keep).sum())
        if not keep.any():
            return None
        xyz, pts = xyz[keep], pts[keep]

        T = self.T[frame]
        body = (T[:3, :3] @ xyz.T).T + T[:3, 3]
        t_start = msg["t"] + time_shift
        t_abs = t_start + (pts["colIdx"].astype(np.float64) / self.n_cols) * self.period
        ring = pts["rowIdx"].astype(np.uint16) + ring_offset
        return (t_start, t_start + self.period, body,
                pts["intensity"].astype(np.float32), ring, t_abs)

    def add_primary(self, msg, time_shift):
        rec = self._prepare(msg, time_shift, self.primary, 0)
        if rec is not None:
            self.pending.append(rec)

    def add_secondary(self, msg, time_shift):
        rec = self._prepare(msg, time_shift, self.secondary, self.RING_OFFSET)
        if rec is not None:
            self.buf.append(rec)

    def _buf_end(self):
        return max((r[1] for r in self.buf), default=-np.inf)

    def _build(self, rec):
        t0, t1, xyz, inten, ring, t_abs = rec
        xs, ins, rs, ts = [xyz], [inten], [ring], [t_abs]
        self.stats["pri_points"] += len(xyz)
        for b in self.buf:
            if b[1] < t0 or b[0] > t1:
                continue
            m = (b[5] >= t0) & (b[5] < t1)
            if not m.any():
                continue
            xs.append(b[2][m]); ins.append(b[3][m]); rs.append(b[4][m]); ts.append(b[5][m])
            self.stats["sec_points"] += int(m.sum())
        xyz = np.concatenate(xs)
        out = np.empty(len(xyz), dtype=PC_DTYPE)
        out["x"], out["y"], out["z"] = (xyz[:, 0].astype(np.float32),
                                       xyz[:, 1].astype(np.float32),
                                       xyz[:, 2].astype(np.float32))
        out["intensity"] = np.concatenate(ins)
        out["ring"] = np.concatenate(rs)
        out["time"] = (np.concatenate(ts) - t0).astype(np.float32)
        self.stats["merged"] += 1
        return t0, out

    def drain(self, flush=False):
        """Yield (stamp, points) for every window whose lidar1 data has arrived."""
        end = self._buf_end()
        while self.pending:
            rec = self.pending[0]
            if not flush and rec[1] > end:
                break
            self.pending.pop(0)
            yield self._build(rec)
            self.buf = [b for b in self.buf if b[1] >= rec[1]]


class ReorderBuffer:
    """Keeps bag writes monotonic in time.

    Merged clouds are released a couple of scans behind the read position, so their
    stamps are older than IMU messages already seen. Everything is pushed here and
    popped in timestamp order once it is far enough behind the read head; the delay
    is bounded, so this holds well under a second of data rather than the whole bag.
    """

    def __init__(self, writer, lag=0.4):
        self.w = writer
        self.lag = lag
        self.heap = []
        self.seq = 0

    def push(self, ts_ns, conn, payload):
        heapq.heappush(self.heap, (ts_ns, self.seq, conn, payload))
        self.seq += 1

    def flush(self, up_to_ns=None):
        while self.heap:
            if up_to_ns is not None and self.heap[0][0] > up_to_ns - self.lag * 1e9:
                break
            ts, _, conn, payload = heapq.heappop(self.heap)
            self.w.write(conn, ts, payload)


def topic_ids(con):
    return {name: tid for tid, name in con.execute("SELECT id, name FROM topics")}


def convert(src: Path, dst: Path, lidars, keep_driveline: bool,
            time_shift: float, beta: float, n_cols: int, period: float,
            duration: float | None, merge: bool = False,
            sensors_yaml: str | None = None, blind: float = 1.0,
            merged_topic: str = "/ouster_merged") -> dict:
    con = sqlite3.connect(f"file:{src}?mode=ro", uri=True)
    ids = topic_ids(con)
    want = {ids[t]: t for t in ([f"/{l}" for l in lidars] + ["/imu", "/gps"] +
                                (["/driveline"] if keep_driveline else []))
            if t in ids}
    if not want:
        raise SystemExit(f"none of the expected topics found in {src}")

    # ---- IMU first: Madgwick needs the whole run, and it is small enough to hold.
    imu_rows = con.execute(
        "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (ids["/imu"],)
    ).fetchall()
    imu = [cdr.decode_imu(bytes(r[0])) for r in imu_rows]
    it = np.array([m["t"] for m in imu])
    ig = np.array([m["gyro"] for m in imu])
    ia = np.array([m["acc"] for m in imu])
    iq = madgwick.run(it, ig, ia, beta=beta)
    imu_quat = {round(t, 6): q for t, q in zip(it, iq)}

    # ---- GPS origin for the local ENU frame.
    gps_rows = con.execute(
        "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (ids["/gps"],)
    ).fetchall() if "/gps" in ids else []
    gps = [cdr.decode_gps(bytes(r[0])) for r in gps_rows]
    valid = [g for g in gps if g["status"] != 0]
    origin = None
    if valid:
        origin = (valid[0]["easting"], valid[0]["northing"], valid[0]["alt"],
                  valid[0]["lat"], valid[0]["lon"])

    t0 = None
    stats = {"lidar": 0, "imu": 0, "gps": 0, "driveline": 0,
             "points_in": 0, "points_out": 0, "empty_scans": 0}

    merger = None
    if merge:
        if len(lidars) != 2:
            raise SystemExit("--merge-lidars needs exactly two --lidars")
        yaml_path = sensors_yaml or str(Path(src).resolve().parents[1] / "sensors.yaml")
        merger = LidarMerger(Extrinsics(yaml_path), f"/{lidars[0]}", f"/{lidars[1]}",
                             period, n_cols, blind)

    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists():
        dst.unlink()

    with Writer(dst) as w:
        conns = {}
        rb = ReorderBuffer(w)

        def conn(topic, typ):
            if topic not in conns:
                conns[topic] = w.add_connection(topic, typ, typestore=TS)
            return conns[topic]

        def emit_merged():
            for stamp, pts in merger.drain():
                msg = make_pointcloud(stamp, "body", pts, stats["lidar"])
                rb.push(int(stamp * 1e9),
                        conn(merged_topic, "sensor_msgs/msg/PointCloud2"),
                        TS.serialize_ros1(msg, "sensor_msgs/msg/PointCloud2"))
                stats["lidar"] += 1
                stats["points_out"] += len(pts)

        cur = con.execute(
            "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp")
        for tid, ts_ns, blob in cur:
            topic = want.get(tid)
            if topic is None:
                continue
            if t0 is None:
                t0 = ts_ns
            if duration is not None and (ts_ns - t0) * 1e-9 > duration:
                break
            data = bytes(blob)

            if topic == "/imu":
                m = cdr.decode_imu(data)
                q = imu_quat.get(round(m["t"], 6), np.array([0.0, 0.0, 0.0, 1.0]))
                msg = make_imu(m["t"], "imu", q, m["gyro"], m["acc"], stats["imu"])
                rb.push(int(m["t"] * 1e9), conn("/imu", "sensor_msgs/msg/Imu"),
                        TS.serialize_ros1(msg, "sensor_msgs/msg/Imu"))
                stats["imu"] += 1

            elif topic == "/gps":
                g = cdr.decode_gps(data)
                msg = make_navsatfix(g["t"], "gps", g, stats["gps"])
                rb.push(int(g["t"] * 1e9), conn("/gps/fix", "sensor_msgs/msg/NavSatFix"),
                        TS.serialize_ros1(msg, "sensor_msgs/msg/NavSatFix"))
                stats["gps"] += 1
                if origin is not None and g["status"] != 0:
                    od = make_gps_odom(g["t"], "enu", "gps",
                                       g["easting"] - origin[0],
                                       g["northing"] - origin[1],
                                       g["alt"] - origin[2], g, stats["gps"])
                    rb.push(int(g["t"] * 1e9), conn("/gps/odom_enu", "nav_msgs/msg/Odometry"),
                            TS.serialize_ros1(od, "nav_msgs/msg/Odometry"))

            elif topic == "/driveline":
                d = cdr.decode_driveline(data)
                msg = make_twist(d["t"], "base_link", d["speed"], d["turn_rate"], stats["driveline"])
                rb.push(int(d["t"] * 1e9), conn("/driveline", "geometry_msgs/msg/TwistStamped"),
                        TS.serialize_ros1(msg, "geometry_msgs/msg/TwistStamped"))
                stats["driveline"] += 1

            else:  # a lidar
                m = cdr.decode_pointcloud2(data)
                stats["points_in"] += len(m["points"])
                if merger is not None:
                    if topic == merger.primary:
                        merger.add_primary(m, time_shift)
                    else:
                        merger.add_secondary(m, time_shift)
                    emit_merged()
                else:
                    res = convert_cloud(m, time_shift, n_cols, period)
                    if res is None:
                        stats["empty_scans"] += 1
                        continue
                    stamp, pts = res
                    msg = make_pointcloud(stamp, m["frame"], pts, stats["lidar"])
                    rb.push(int(stamp * 1e9),
                            conn(topic, "sensor_msgs/msg/PointCloud2"),
                            TS.serialize_ros1(msg, "sensor_msgs/msg/PointCloud2"))
                    stats["lidar"] += 1
                    stats["points_out"] += len(pts)

            rb.flush(ts_ns)

        if merger is not None:
            for stamp, pts in merger.drain(flush=True):
                msg = make_pointcloud(stamp, "body", pts, stats["lidar"])
                rb.push(int(stamp * 1e9),
                        conn(merged_topic, "sensor_msgs/msg/PointCloud2"),
                        TS.serialize_ros1(msg, "sensor_msgs/msg/PointCloud2"))
                stats["lidar"] += 1
                stats["points_out"] += len(pts)
            stats["merge"] = merger.stats
        rb.flush()

    con.close()
    stats["origin"] = origin
    return stats


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src", help="path to data.db3")
    ap.add_argument("dst", help="output .bag path")
    ap.add_argument("--lidars", default="ouster0",
                    help="comma-separated lidar topics without the slash "
                         "(default: ouster0; FAST-LIO takes one)")
    ap.add_argument("--time-shift", type=float, default=DEFAULT_TIME_SHIFT,
                    help="seconds added to each lidar stamp so that the emitted "
                         "stamp marks the true scan start. See "
                         "determine_stamp_convention.py; default %(default)s")
    ap.add_argument("--beta", type=float, default=0.05, help="Madgwick gain")
    ap.add_argument("--n-cols", type=int, default=N_COLS)
    ap.add_argument("--period", type=float, default=LIDAR_PERIOD)
    ap.add_argument("--no-driveline", action="store_true")
    ap.add_argument("--duration", type=float, default=None,
                    help="only convert the first N seconds (for quick tests)")
    ap.add_argument("--merge-lidars", action="store_true",
                    help="merge both lidars into one cloud in the IMU/body frame "
                         "(use with --lidars ouster0,ouster1 and config/charlie8_dual.yaml)")
    ap.add_argument("--merged-topic", default="/ouster_merged")
    ap.add_argument("--sensors-yaml", default=None,
                    help="path to sensors.yaml (default: alongside the dataset)")
    ap.add_argument("--blind", type=float, default=1.5,
                    help="per-lidar self-return cutoff in metres, applied in each "
                         "sensor frame before merging (default %(default)s). "
                         "Matches `blind` in charlie8.yaml on purpose: the single-lidar "
                         "path lets FAST-LIO apply 1.5 m in the lidar frame, so using a "
                         "different value here would mean the dual run also saw a "
                         "different amount of near-field data, confounding the "
                         "comparison. A body-frame radius cannot substitute -- the IMU "
                         "origin is ~1.4 m from each sensor, so one radius there "
                         "corresponds to no consistent range from either lidar.")
    args = ap.parse_args()

    stats = convert(Path(args.src), Path(args.dst),
                    [s.strip() for s in args.lidars.split(",") if s.strip()],
                    not args.no_driveline, args.time_shift, args.beta,
                    args.n_cols, args.period, args.duration,
                    merge=args.merge_lidars, sensors_yaml=args.sensors_yaml,
                    blind=args.blind, merged_topic=args.merged_topic)

    dropped = stats["points_in"] - stats["points_out"]
    pct = 100.0 * dropped / stats["points_in"] if stats["points_in"] else 0.0
    print(f"wrote {args.dst}")
    print(f"  lidar scans   {stats['lidar']}  ({stats['empty_scans']} empty skipped)")
    print(f"  imu           {stats['imu']}")
    print(f"  gps           {stats['gps']}")
    print(f"  driveline     {stats['driveline']}")
    print(f"  points        {stats['points_out']} kept, {dropped} non-finite dropped ({pct:.2f}%)")
    if "merge" in stats:
        ms = stats["merge"]
        tot = ms["pri_points"] + ms["sec_points"]
        print(f"  merged scans  {ms['merged']}  "
              f"lidar0 {ms['pri_points']} pts + lidar1 {ms['sec_points']} pts "
              f"({100.0 * ms['sec_points'] / max(tot, 1):.1f}% from lidar1)")
        print(f"  blind cut     {ms['dropped_blind']} pts within {args.blind} m of a sensor")
    if stats["origin"]:
        e, n, a, lat, lon = stats["origin"]
        print(f"  ENU origin    lat={lat:.7f} lon={lon:.7f} alt={a:.3f} "
              f"(UTM {e:.3f}, {n:.3f})")
    else:
        print("  ENU origin    none - no valid GPS fix in this sequence")


if __name__ == "__main__":
    main()
