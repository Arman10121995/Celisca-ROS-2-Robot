"""Stable geom order for Go2 support-force telemetry."""

GO2_SUPPORT_GEOM_NAMES = tuple(
    f"{leg}_{part}_contact_0"
    for leg in ("FL", "FR", "RL", "RR")
    for part in ("hip", "thigh", "calf", "foot")
) + ("trunk_contact_0", "imu_link_contact_0")