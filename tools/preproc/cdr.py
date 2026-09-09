#!/usr/bin/env python3
"""
Minimal CDR reader for the Charlie8 .db3 bags.

The bags are rosbag2 v9 / sqlite3 and carry two custom types
(prs_sensor_msgs/msg/Gps, prs_sensor_msgs/msg/Driveline). Their full definitions
are embedded in the bag's `message_definitions` table, so nothing outside the
bag is required to decode them -- which is why this module exists instead of a
ROS dependency. It also sidesteps a packaging quirk: only ditches-Charlie8 ships
a metadata.yaml, and the info.yaml the other sequences carry names a bag file
(merged_sorted_bag_0.db3) that is not the one on disk, so a stock rosbag2 reader
cannot open them without repair.

Only the subset of CDR needed here is implemented: primitives, strings, and the
handful of message layouts we consume.
"""
from __future__ import annotations

import struct

import numpy as np

# PointCloud2 field datatype enum -> numpy dtype
PF_DTYPE = {
    1: "i1", 2: "u1", 3: "i2", 4: "u2",
    5: "i4", 6: "u4", 7: "f4", 8: "f8",
}


class CDRReader:
    """Little/big-endian aware CDR stream reader with 4-byte encapsulation header."""

    __slots__ = ("b", "little", "p", "base")

    def __init__(self, buf: bytes):
        self.b = buf
        self.little = buf[1] == 1
        self.p = 4
        self.base = 4

    def align(self, n: int) -> None:
        off = (self.p - self.base) % n
        if off:
            self.p += n - off

    def _unpack(self, fmt: str, size: int, align: int):
        self.align(align)
        v = struct.unpack_from(("<" if self.little else ">") + fmt, self.b, self.p)[0]
        self.p += size
        return v

    def u8(self) -> int:
        v = self.b[self.p]
        self.p += 1
        return v

    def i32(self) -> int:
        return self._unpack("i", 4, 4)

    def u32(self) -> int:
        return self._unpack("I", 4, 4)

    def f32(self) -> float:
        return self._unpack("f", 4, 4)

    def f64(self) -> float:
        return self._unpack("d", 8, 8)

    def string(self) -> str:
        n = self.u32()
        s = self.b[self.p:self.p + n - 1].decode("utf-8", "replace") if n else ""
        self.p += n
        return s

    def header(self):
        """std_msgs/Header -> (stamp_seconds, frame_id)."""
        sec = self.i32()
        nsec = self.u32()
        return sec + nsec * 1e-9, self.string()


def decode_imu(buf: bytes) -> dict:
    c = CDRReader(buf)
    t, frame = c.header()
    quat = [c.f64() for _ in range(4)]
    [c.f64() for _ in range(9)]                       # orientation_covariance
    gyro = [c.f64() for _ in range(3)]
    [c.f64() for _ in range(9)]                       # angular_velocity_covariance
    acc = [c.f64() for _ in range(3)]
    [c.f64() for _ in range(9)]                       # linear_acceleration_covariance
    return {"t": t, "frame": frame, "quat": quat, "gyro": gyro, "acc": acc}


def decode_gps(buf: bytes) -> dict:
    c = CDRReader(buf)
    t, frame = c.header()
    return {
        "t": t, "frame": frame,
        "lat": c.f64(), "lon": c.f64(), "alt": c.f64(),
        "easting": c.f64(), "northing": c.f64(),
        "lat_err": c.f64(), "lon_err": c.f64(),
        "speed": c.f64(), "status": c.i32(),
    }


def decode_driveline(buf: bytes) -> dict:
    c = CDRReader(buf)
    t, frame = c.header()
    return {"t": t, "frame": frame, "speed": c.f32(), "turn_rate": c.f32()}


def decode_pose(buf: bytes) -> dict:
    c = CDRReader(buf)
    t, frame = c.header()
    return {"t": t, "frame": frame,
            "pos": [c.f64() for _ in range(3)],
            "quat": [c.f64() for _ in range(4)]}


def decode_pointcloud2(buf: bytes) -> dict:
    """Returns the header, dimensions, field layout and the points as a
    structured numpy array (a view onto the original buffer)."""
    c = CDRReader(buf)
    t, frame = c.header()
    height = c.u32()
    width = c.u32()
    nfields = c.u32()
    fields = []
    for _ in range(nfields):
        name = c.string()
        offset = c.u32()
        dtype = c.u8()
        count = c.u32()
        fields.append((name, offset, dtype, count))
    is_bigendian = c.u8()
    point_step = c.u32()
    row_step = c.u32()
    nbytes = c.u32()
    data = c.b[c.p:c.p + nbytes]
    c.p += nbytes
    c.align(1)
    is_dense = c.u8() if c.p < len(c.b) else 1

    endian = ">" if is_bigendian else "<"
    names, formats, offsets = [], [], []
    for name, offset, dtype, count in fields:
        if dtype not in PF_DTYPE or count != 1:
            continue
        names.append(name)
        formats.append(endian + PF_DTYPE[dtype])
        offsets.append(offset)
    dt = np.dtype({"names": names, "formats": formats,
                   "offsets": offsets, "itemsize": point_step})
    npts = height * width
    points = np.frombuffer(data[:npts * point_step], dtype=dt) if npts else np.empty(0, dt)

    return {"t": t, "frame": frame, "height": height, "width": width,
            "fields": fields, "point_step": point_step, "row_step": row_step,
            "is_dense": bool(is_dense), "points": points}
