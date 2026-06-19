from pathlib import Path

from .schema import (
    ArmSettings,
    CalibrationMetadata,
    ConfigSource,
    FrequencySettings,
    MotorOverrideSettings,
    MotorProfileSettings,
    PIDSettings,
    SOARMConfig,
)


def default_config_path() -> Path:
    """Return the packaged default SOARM SDK arm config."""

    return Path(__file__).resolve().parents[1] / "configs" / "soarm-sdk.yaml"


__all__ = [
    "ArmSettings",
    "CalibrationMetadata",
    "ConfigSource",
    "FrequencySettings",
    "MotorOverrideSettings",
    "MotorProfileSettings",
    "PIDSettings",
    "SOARMConfig",
    "default_config_path",
]
