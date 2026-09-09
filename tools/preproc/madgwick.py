#!/usr/bin/env python3
"""
Madgwick AHRS (IMU-only, no magnetometer) to regenerate the IMU orientation.

Why this exists
---------------
The `orientation` quaternion recorded on /imu in the .db3 bags is unusable.
Rotating the measured specific force by it should return [0, 0, +9.81] at rest;
instead it returns e.g. [8.91, 3.36, 2.39] (niceFeatures) and is not even
self-consistent in ditches (sigma ~ 1.3 m/s^2). All three covariance blocks are
zero. The gyro and accelerometer measurements themselves are healthy -- exactly
100 Hz, |acc| = 9.823 -- so only the orientation needed replacing.

This reproduces the /imu_filtered topic already present in the ROS1
data_ros1_radaronly.bag files, whose gyro/accel are byte-identical to /imu.
Regenerating it here keeps the pipeline reproducible from the .db3 alone, and
lets the released dataset ship a corrected orientation.

Frame convention: IMU is x-right, y-forward, z-up, so at rest the accelerometer
reads +g on z. That matches Madgwick's reference direction of gravity, which is
what the objective function below assumes.

Note the SLAM front-end does not consume this quaternion -- FAST-LIO variants
integrate raw gyro/accel and estimate gravity themselves. It is produced for the
dataset release, for initial-attitude sanity checks, and for anyone who wants a
drop-in replacement for the broken field.
"""
from __future__ import annotations

import numpy as np


def _normalise(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


class MadgwickAHRS:
    """Sequential Madgwick filter. State is a quaternion [w, x, y, z]."""

    def __init__(self, beta: float = 0.05, q0=None):
        self.beta = float(beta)
        self.q = np.array([1.0, 0.0, 0.0, 0.0]) if q0 is None else np.asarray(q0, float)

    def update(self, gyro, accel, dt: float) -> np.ndarray:
        """One step. gyro in rad/s, accel in m/s^2 (any scale), dt in seconds."""
        q = self.q
        gx, gy, gz = gyro

        # Rate of change from the gyroscope alone.
        qdot = 0.5 * np.array([
            -q[1] * gx - q[2] * gy - q[3] * gz,
             q[0] * gx + q[2] * gz - q[3] * gy,
             q[0] * gy - q[1] * gz + q[3] * gx,
             q[0] * gz + q[1] * gy - q[2] * gx,
        ])

        a = np.asarray(accel, float)
        if np.linalg.norm(a) > 0:
            ax, ay, az = _normalise(a)
            qw, qx, qy, qz = q

            # Objective: rotate the earth-frame gravity direction [0,0,1] into the
            # body frame and compare against the normalised accelerometer reading.
            f = np.array([
                2.0 * (qx * qz - qw * qy) - ax,
                2.0 * (qw * qx + qy * qz) - ay,
                2.0 * (0.5 - qx * qx - qy * qy) - az,
            ])
            J = np.array([
                [-2.0 * qy,  2.0 * qz, -2.0 * qw, 2.0 * qx],
                [ 2.0 * qx,  2.0 * qw,  2.0 * qz, 2.0 * qy],
                [ 0.0,      -4.0 * qx, -4.0 * qy, 0.0     ],
            ])
            grad = _normalise(J.T @ f)
            qdot -= self.beta * grad

        q = q + qdot * dt
        self.q = _normalise(q)
        return self.q

    @property
    def quaternion_xyzw(self) -> np.ndarray:
        """ROS ordering."""
        w, x, y, z = self.q
        return np.array([x, y, z, w])


def initial_quaternion(accel: np.ndarray) -> np.ndarray:
    """Level the filter from a mean accelerometer reading (yaw left at zero).

    Returns [w, x, y, z]. Builds the shortest rotation carrying the measured
    gravity direction onto +z, which removes the long settling transient the
    filter would otherwise show at the start of a run.
    """
    a = _normalise(np.asarray(accel, float))
    up = np.array([0.0, 0.0, 1.0])
    v = np.cross(a, up)
    c = float(np.dot(a, up))
    if np.linalg.norm(v) < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0]) if c > 0 else np.array([0.0, 1.0, 0.0, 0.0])
    axis = _normalise(v)
    angle = math_acos_clamped(c)
    s = np.sin(angle / 2.0)
    return np.array([np.cos(angle / 2.0), axis[0] * s, axis[1] * s, axis[2] * s])


def math_acos_clamped(c: float) -> float:
    return float(np.arccos(np.clip(c, -1.0, 1.0)))


def run(timestamps, gyros, accels, beta: float = 0.05, settle: int = 200) -> np.ndarray:
    """Filter a whole run. Returns an (N, 4) array of [x, y, z, w] quaternions.

    `settle` samples at the start are used to level the initial attitude, then
    replayed so every input sample gets an output.
    """
    t = np.asarray(timestamps, float)
    g = np.asarray(gyros, float)
    a = np.asarray(accels, float)
    if not (len(t) == len(g) == len(a)):
        raise ValueError("timestamps, gyros and accels must be the same length")

    n_settle = min(settle, len(a))
    filt = MadgwickAHRS(beta=beta, q0=initial_quaternion(a[:n_settle].mean(axis=0)))

    dts = np.diff(t, prepend=t[0] - (t[1] - t[0] if len(t) > 1 else 0.01))
    # Guard against duplicate or out-of-order stamps.
    median_dt = float(np.median(dts[1:])) if len(dts) > 1 else 0.01
    dts = np.where((dts <= 0) | (dts > 10 * median_dt), median_dt, dts)

    out = np.empty((len(t), 4))
    for i in range(len(t)):
        filt.update(g[i], a[i], dts[i])
        out[i] = filt.quaternion_xyzw
    return out
