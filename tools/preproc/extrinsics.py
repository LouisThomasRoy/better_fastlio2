#!/usr/bin/env python3
"""
Resolve sensors.yaml into the transforms the SLAM pipeline needs.

Vehicle frame convention (derived from sensors.yaml geometry and verified against
lidar ground-plane fits):

    x = right, y = forward, z = up      (right-handed, but NOT ROS REP-103)

    Evidence: radar2 is the front-centre unit at y=+1.50 with zero yaw; radar0/1
    sit at x=+/-0.76 with -/+60 deg yaw, i.e. angled outward from forward.
    Applying these extrinsics puts the lidar ground plane at z = 0 +/- 0.07 m on
    the flat sequences, so the vehicle origin is at ground level.

euler_vehicle_sensor is [roll_x, pitch_y, yaw_z], applied as Rz(yaw) Ry(pitch) Rx(roll).

The IMU-to-vehicle rotation is identity. sensors.yaml records it as "unknown", but:
  * accel reads [~0.1, ~-0.07, 9.82] at rest -> z is up, level within ~0.6 deg
  * corr(gyro_z, driveline turn_rate) = -0.93
  * the forward axis sits 90.1 deg from IMU +x -> IMU +y is vehicle forward
  * radar0_config.yaml already assumes it: its l_b_r equals p_radar0 - p_imu exactly

FAST-LIO variants want the lidar pose expressed in the IMU frame (T_imu_lidar),
which is what `sensor_in_imu` returns.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import yaml

# Sensors whose euler triple is absent or meaningless in sensors.yaml.
_NO_ROTATION = {"gps"}


def euler_to_matrix(rpy) -> np.ndarray:
    """Rz(yaw) @ Ry(pitch) @ Rx(roll) from [roll, pitch, yaw] in radians."""
    r, p, y = (float(v) for v in rpy)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def matrix_to_quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> quaternion [x, y, z, w]."""
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


class Extrinsics:
    """sensors.yaml -> transforms, with the IMU as the body frame."""

    def __init__(self, sensors_yaml: str | Path):
        with open(sensors_yaml) as fh:
            self.raw = yaml.safe_load(fh)
        if "imu" not in self.raw:
            raise KeyError(f"no 'imu' entry in {sensors_yaml}")
        self.p_imu = np.asarray(self.raw["imu"]["p_vehicle2sensor_vehicle"], dtype=float)

    def names(self):
        return [k for k in self.raw if isinstance(self.raw[k], dict)]

    def sensor_in_vehicle(self, name: str):
        """(R_vehicle_sensor, t_vehicle_sensor)."""
        entry = self.raw[name]
        t = np.asarray(entry["p_vehicle2sensor_vehicle"], dtype=float)
        rpy = entry.get("euler_vehicle_sensor")
        if name in _NO_ROTATION or not isinstance(rpy, (list, tuple)):
            # Either genuinely rotation-free (GPS antenna) or recorded as "unknown".
            # For the IMU "unknown" resolves to identity -- see module docstring.
            R = np.eye(3)
        else:
            R = euler_to_matrix(rpy)
        return R, t

    def sensor_in_imu(self, name: str):
        """(R_imu_sensor, t_imu_sensor). R_vehicle_imu is identity, so the
        rotation carries over unchanged and the translation is a subtraction."""
        R, t = self.sensor_in_vehicle(name)
        return R, t - self.p_imu

    def homogeneous(self, name: str) -> np.ndarray:
        R, t = self.sensor_in_imu(name)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        return T

    def fastlio_block(self, name: str) -> str:
        """extrinsic_T / extrinsic_R lines for a FAST-LIO style config."""
        R, t = self.sensor_in_imu(name)
        rows = ",\n                   ".join(
            ", ".join(f"{v: .9f}" for v in row) for row in R
        )
        return (
            f"    extrinsic_T: [ {t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f} ]\n"
            f"    extrinsic_R: [ {rows} ]"
        )

    def report(self) -> str:
        lines = [
            "Translations expressed in the IMU frame (x-right, y-forward, z-up)",
            f"  IMU origin in vehicle frame: {self.p_imu.tolist()}",
            "",
            f"  {'sensor':10s} {'x':>10s} {'y':>10s} {'z':>10s}   "
            f"{'qx':>9s} {'qy':>9s} {'qz':>9s} {'qw':>9s}",
        ]
        for name in self.names():
            R, t = self.sensor_in_imu(name)
            q = matrix_to_quat(R)
            lines.append(
                f"  {name:10s} {t[0]:10.4f} {t[1]:10.4f} {t[2]:10.4f}   "
                f"{q[0]:9.6f} {q[1]:9.6f} {q[2]:9.6f} {q[3]:9.6f}"
            )
        lines += [
            "",
            "  Cross-check: the radar0 row must equal l_b_r in radar0_config.yaml",
            "  ([0.76, 1.22, 0.1275], q_z=-0.4999145, q_w=0.8660748).",
        ]
        return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sensors_yaml", nargs="?",
                    default=str(Path(__file__).resolve().parents[1] / "sensors.yaml"))
    ap.add_argument("--fastlio", metavar="SENSOR",
                    help="emit FAST-LIO extrinsic_T/extrinsic_R for this sensor")
    args = ap.parse_args()

    ex = Extrinsics(args.sensors_yaml)
    if args.fastlio:
        print(ex.fastlio_block(args.fastlio))
    else:
        print(ex.report())


if __name__ == "__main__":
    main()
