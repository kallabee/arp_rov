"""Euler-angle attitude filters (EKF / ESKF with gyro bias / UKF sigma-point).

These are simplified references intended for low-rate prototyping; prefer
``madgwick`` or ``complementary`` for robust field use on small platforms.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Tuple

import numpy as np

from imu_navigation.types import Dof9Sample


def _wrap_pi(a: float) -> float:
    return float(np.arctan2(np.sin(a), np.cos(a)))


def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)


def _R_euler(r: float, p: float, y: float) -> np.ndarray:
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return Rz @ Ry @ Rx


def _accel_predict(r: float, p: float, y: float, g: np.ndarray) -> np.ndarray:
    R = _R_euler(r, p, y)
    return R.T @ g


def _numeric_H_accel(r: float, p: float, y: float, g: np.ndarray, eps: float = 1e-4) -> np.ndarray:
    z0 = _accel_predict(r, p, y, g)
    H = np.zeros((3, 3))
    for i, d in enumerate((eps, eps, eps)):
        e = np.zeros(3)
        e[i] = d
        zp = _accel_predict(r + e[0], p + e[1], y + e[2], g)
        H[:, i] = (zp - z0) / d
    return H


def _mag_predict(r: float, p: float, y: float, m_w: np.ndarray) -> np.ndarray:
    R = _R_euler(r, p, y)
    return R.T @ m_w


@dataclass
class IndirectEKF:
    g_world: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 9.81]))
    mag_world: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.3]))
    sigma_gyro: float = 0.02
    sigma_accel: float = 0.4
    sigma_mag: float = 0.15
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    P: np.ndarray = field(default_factory=lambda: np.eye(3) * 0.1)

    def update(self, s: Dof9Sample, dt: float) -> tuple[float, float, float, float]:
        gx, gy, gz = float(s.gyro_x), float(s.gyro_y), float(s.gyro_z)
        self.roll = _wrap_pi(self.roll + gx * dt)
        self.pitch = _wrap_pi(self.pitch + gy * dt)
        self.yaw = _wrap_pi(self.yaw + gz * dt)

        F = np.eye(3)
        q = (self.sigma_gyro**2) * dt * dt
        self.P = F @ self.P @ F.T + np.eye(3) * max(q, 1e-10)

        a = np.array([s.accel_x, s.accel_y, s.accel_z], dtype=float)
        na = np.linalg.norm(a)
        if na > 1e-6:
            z = a / na
            h = _accel_predict(self.roll, self.pitch, self.yaw, self.g_world)
            nh = np.linalg.norm(h)
            if nh > 1e-6:
                h = h / nh
            y_in = z - h
            H = _numeric_H_accel(self.roll, self.pitch, self.yaw, self.g_world)
            Rm = np.eye(3) * (self.sigma_accel**2)
            S = H @ self.P @ H.T + Rm
            K = self.P @ H.T @ np.linalg.inv(S)
            dx = K @ y_in
            self.roll = _wrap_pi(self.roll + float(dx[0]))
            self.pitch = _wrap_pi(self.pitch + float(dx[1]))
            self.yaw = _wrap_pi(self.yaw + float(dx[2]))
            self.P = (np.eye(3) - K @ H) @ self.P

        m = np.array([s.mag_x, s.mag_y, s.mag_z], dtype=float)
        nm = np.linalg.norm(m)
        if nm > 1e-6 and np.linalg.norm(self.mag_world) > 1e-6:
            z = m / nm
            mw = self.mag_world / np.linalg.norm(self.mag_world)
            h = _mag_predict(self.roll, self.pitch, self.yaw, mw)
            y_in = z - h
            H = np.zeros((3, 3))
            eps = 1e-4
            h0 = h.copy()
            for i in range(3):
                e = np.zeros(3)
                e[i] = eps
                hp = _mag_predict(self.roll + e[0], self.pitch + e[1], self.yaw + e[2], mw) - h0
                H[:, i] = hp / eps
            Rm = np.eye(3) * (self.sigma_mag**2)
            S = H @ self.P @ H.T + Rm
            K = self.P @ H.T @ np.linalg.inv(S)
            dx = K @ y_in
            self.roll = _wrap_pi(self.roll + float(dx[0]))
            self.pitch = _wrap_pi(self.pitch + float(dx[1]))
            self.yaw = _wrap_pi(self.yaw + float(dx[2]))
            self.P = (np.eye(3) - K @ H) @ self.P

        return _quat_from_rpy(self.roll, self.pitch, self.yaw)


@dataclass
class IndirectESKF:
    """EKF with a random-walk gyro bias (6 states)."""

    g_world: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 9.81]))
    mag_world: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.3]))
    sigma_gyro: float = 0.02
    sigma_bias: float = 1e-4
    sigma_accel: float = 0.4
    sigma_mag: float = 0.15
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    bg: np.ndarray = field(default_factory=lambda: np.zeros(3))
    P: np.ndarray = field(default_factory=lambda: np.eye(6) * 0.05)

    def update(self, s: Dof9Sample, dt: float) -> tuple[float, float, float, float]:
        gx = np.array([s.gyro_x, s.gyro_y, s.gyro_z], dtype=float) - self.bg
        self.roll = _wrap_pi(self.roll + float(gx[0]) * dt)
        self.pitch = _wrap_pi(self.pitch + float(gx[1]) * dt)
        self.yaw = _wrap_pi(self.yaw + float(gx[2]) * dt)

        F = np.eye(6)
        F[0, 3] = -dt
        F[1, 4] = -dt
        F[2, 5] = -dt
        Q = np.eye(6)
        Q[:3, :3] *= (self.sigma_gyro**2) * dt * dt
        Q[3:, 3:] *= (self.sigma_bias**2) * dt
        self.P = F @ self.P @ F.T + Q

        a = np.array([s.accel_x, s.accel_y, s.accel_z], dtype=float)
        na = np.linalg.norm(a)
        if na > 1e-6:
            z = a / na
            h = _accel_predict(self.roll, self.pitch, self.yaw, self.g_world)
            nh = np.linalg.norm(h)
            if nh > 1e-6:
                h = h / nh
            y_in = z - h
            H3 = _numeric_H_accel(self.roll, self.pitch, self.yaw, self.g_world)
            H = np.zeros((3, 6))
            H[:, :3] = H3
            Rm = np.eye(3) * (self.sigma_accel**2)
            S = H @ self.P @ H.T + Rm
            K = self.P @ H.T @ np.linalg.inv(S)
            dx = K @ y_in
            self.roll = _wrap_pi(self.roll + float(dx[0]))
            self.pitch = _wrap_pi(self.pitch + float(dx[1]))
            self.yaw = _wrap_pi(self.yaw + float(dx[2]))
            self.bg = self.bg + dx[3:6]
            self.P = (np.eye(6) - K @ H) @ self.P

        m = np.array([s.mag_x, s.mag_y, s.mag_z], dtype=float)
        nm = np.linalg.norm(m)
        if nm > 1e-6 and np.linalg.norm(self.mag_world) > 1e-6:
            z = m / nm
            mw = self.mag_world / np.linalg.norm(self.mag_world)
            h0 = _mag_predict(self.roll, self.pitch, self.yaw, mw)
            y_in = z - h0
            H3 = np.zeros((3, 3))
            eps = 1e-4
            for i in range(3):
                e = np.zeros(3)
                e[i] = eps
                H3[:, i] = (_mag_predict(self.roll + e[0], self.pitch + e[1], self.yaw + e[2], mw) - h0) / eps
            H = np.zeros((3, 6))
            H[:, :3] = H3
            Rm = np.eye(3) * (self.sigma_mag**2)
            S = H @ self.P @ H.T + Rm
            K = self.P @ H.T @ np.linalg.inv(S)
            dx = K @ y_in
            self.roll = _wrap_pi(self.roll + float(dx[0]))
            self.pitch = _wrap_pi(self.pitch + float(dx[1]))
            self.yaw = _wrap_pi(self.yaw + float(dx[2]))
            self.bg = self.bg + dx[3:6]
            self.P = (np.eye(6) - K @ H) @ self.P

        return _quat_from_rpy(self.roll, self.pitch, self.yaw)


@dataclass
class IndirectUKF:
    """Additive UKF on ``[roll, pitch, yaw]`` with vector accel + mag observations."""

    g_world: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 9.81]))
    mag_world: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.3]))
    alpha: float = 0.08
    beta: float = 2.0
    kappa: float = 0.0
    sigma_gyro: float = 0.02
    sigma_accel: float = 0.4
    sigma_mag: float = 0.15
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    P: np.ndarray = field(default_factory=lambda: np.eye(3) * 0.1)

    def _sigma_points(self, x: np.ndarray, P: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        n = 3
        lam = self.alpha**2 * (n + self.kappa) - n
        gamma = math.sqrt(n + lam)
        try:
            L = np.linalg.cholesky(P)
        except np.linalg.LinAlgError:
            L = np.sqrt(np.maximum(np.diag(P), 1e-8)) * np.eye(3)
        chi = [x.copy()]
        for i in range(n):
            chi.append(x + gamma * L[:, i])
            chi.append(x - gamma * L[:, i])
        Wm = [lam / (n + lam)]
        Wc = [lam / (n + lam) + (1.0 - self.alpha**2 + self.beta)]
        wi = 1.0 / (2.0 * (n + lam))
        for _ in range(2 * n):
            Wm.append(wi)
            Wc.append(wi)
        return np.stack(chi), np.array(Wm), np.array(Wc)

    def _ukf_update_vec(
        self,
        z_meas: np.ndarray,
        chi: np.ndarray,
        Wm: np.ndarray,
        Wc: np.ndarray,
        pred_fn,
        meas_noise: float,
    ) -> None:
        n_sig = chi.shape[0]
        Z = np.zeros((3, n_sig))
        for i in range(n_sig):
            r, p, y = float(chi[i, 0]), float(chi[i, 1]), float(chi[i, 2])
            Z[:, i] = pred_fn(r, p, y)
        z_mean = Z @ Wm
        Pzz = np.zeros((3, 3))
        Pxz = np.zeros((3, 3))
        x0 = np.array([self.roll, self.pitch, self.yaw], dtype=float)
        for i in range(n_sig):
            dz = Z[:, i] - z_mean
            dx = chi[i] - x0
            Pzz += float(Wc[i]) * np.outer(dz, dz)
            Pxz += float(Wc[i]) * np.outer(dx, dz)
        Pzz += np.eye(3) * (meas_noise**2)
        K = Pxz @ np.linalg.inv(Pzz)
        innov = z_meas - z_mean
        dx = K @ innov
        self.roll = _wrap_pi(self.roll + float(dx[0]))
        self.pitch = _wrap_pi(self.pitch + float(dx[1]))
        self.yaw = _wrap_pi(self.yaw + float(dx[2]))
        self.P = self.P - K @ Pzz @ K.T

    def update(self, s: Dof9Sample, dt: float) -> tuple[float, float, float, float]:
        gx, gy, gz = float(s.gyro_x), float(s.gyro_y), float(s.gyro_z)
        self.roll = _wrap_pi(self.roll + gx * dt)
        self.pitch = _wrap_pi(self.pitch + gy * dt)
        self.yaw = _wrap_pi(self.yaw + gz * dt)

        q = (self.sigma_gyro**2) * dt * dt
        self.P = self.P + np.eye(3) * max(q, 1e-10)

        x = np.array([self.roll, self.pitch, self.yaw], dtype=float)
        chi, Wm, Wc = self._sigma_points(x, self.P)

        a = np.array([s.accel_x, s.accel_y, s.accel_z], dtype=float)
        na = np.linalg.norm(a)
        if na > 1e-6:

            def ap(r: float, p: float, y: float) -> np.ndarray:
                h = _accel_predict(r, p, y, self.g_world)
                nh = np.linalg.norm(h)
                return h / nh if nh > 1e-6 else h

            z_meas = a / na
            self._ukf_update_vec(z_meas, chi, Wm, Wc, ap, self.sigma_accel)
            x = np.array([self.roll, self.pitch, self.yaw], dtype=float)
            chi, Wm, Wc = self._sigma_points(x, self.P)

        m = np.array([s.mag_x, s.mag_y, s.mag_z], dtype=float)
        nm = np.linalg.norm(m)
        if nm > 1e-6 and np.linalg.norm(self.mag_world) > 1e-6:

            def mp(r: float, p: float, y: float) -> np.ndarray:
                mw = self.mag_world / np.linalg.norm(self.mag_world)
                h = _mag_predict(r, p, y, mw)
                nh = np.linalg.norm(h)
                return h / nh if nh > 1e-6 else h

            z_meas = m / nm
            self._ukf_update_vec(z_meas, chi, Wm, Wc, mp, self.sigma_mag)

        return _quat_from_rpy(self.roll, self.pitch, self.yaw)
