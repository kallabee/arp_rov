from __future__ import annotations

from typing import Any, Protocol


class OrientationFilter(Protocol):
    def update(self, sample: Any, dt: float) -> tuple[float, float, float, float]: ...


def make_orientation_filter(name: str, cfg: dict) -> OrientationFilter:
    n = (name or "complementary").lower().strip()
    if n == "complementary":
        from imu_navigation.filters.complementary import ComplementaryFilter

        return ComplementaryFilter(alpha=float(cfg.get("complementary_alpha", 0.98)))
    if n == "madgwick":
        from imu_navigation.filters.madgwick import MadgwickFilter

        return MadgwickFilter(beta=float(cfg.get("madgwick_beta", 0.1)))
    if n == "ekf":
        from imu_navigation.filters.indirect_kalman import IndirectEKF

        return IndirectEKF()
    if n == "eskf":
        from imu_navigation.filters.indirect_kalman import IndirectESKF

        return IndirectESKF()
    if n == "ukf":
        from imu_navigation.filters.indirect_kalman import IndirectUKF

        return IndirectUKF(
            alpha=float(cfg.get("ukf_alpha", 0.08)),
            beta=float(cfg.get("ukf_beta", 2.0)),
            kappa=float(cfg.get("ukf_kappa", 0.0)),
        )
    raise ValueError(f"Unknown orientation filter: {name!r} (supported: complementary, madgwick, ekf, ukf, eskf)")
