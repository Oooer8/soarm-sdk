from __future__ import annotations

from pathlib import Path
from shutil import copy2

from ..errors import ConfigurationError


PACKAGE_CONFIG_DIR = Path(__file__).resolve().parents[1] / "configs"
PACKAGED_CONFIG_PATH = PACKAGE_CONFIG_DIR / "soarm-sdk.yaml"
USER_CONFIG_PATH = Path("configs") / "soarm-sdk.yaml"


def default_config_path() -> Path:
    """Return the packaged canonical SOARM SDK config template."""

    return PACKAGED_CONFIG_PATH


def default_user_config_path() -> Path:
    """Return the default generated user config path."""

    return USER_CONFIG_PATH


def ensure_user_config(
    path: str | Path | None = None,
    *,
    overwrite: bool = False,
) -> Path:
    """Copy the packaged split config set to a user-editable config directory.

    The SDK keeps one canonical config set under ``soarm_sdk/configs`` for
    packaging. Runtime edits such as ports, calibration, and Web settings
    belong in a generated user copy.
    """

    destination = Path(path) if path is not None else default_user_config_path()
    destination.parent.mkdir(parents=True, exist_ok=True)
    if overwrite or not destination.exists():
        _copy_required(PACKAGED_CONFIG_PATH, destination)
    runtime_destination = destination.parent / "runtime.yaml"
    if overwrite or not runtime_destination.exists():
        _copy_required(PACKAGE_CONFIG_DIR / "runtime.yaml", runtime_destination)
    motors_dir = destination.parent / "motors"
    motors_dir.mkdir(parents=True, exist_ok=True)
    motor_destination = motors_dir / "feetech_sts3215.yaml"
    if overwrite or not motor_destination.exists():
        _copy_required(PACKAGE_CONFIG_DIR / "motors" / "feetech_sts3215.yaml", motor_destination)
    return destination


def resolve_config_path(
    path: str | Path | None = None,
    *,
    for_write: bool = False,
) -> Path:
    """Resolve a config path for SDK commands.

    Explicit paths are always honored. Without an explicit path, commands use an
    existing user copy when present, otherwise the packaged canonical config.
    Commands that write configuration create the user copy first.
    """

    if path is not None:
        explicit = Path(path)
        if for_write and not explicit.exists():
            return ensure_user_config(explicit)
        return explicit
    if for_write:
        return ensure_user_config()
    user_config = default_user_config_path()
    if user_config.exists():
        return user_config
    return default_config_path()


def _copy_required(source: Path, destination: Path) -> None:
    try:
        copy2(source, destination)
    except OSError as exc:
        raise ConfigurationError(
            f"Failed to copy default config {source} to {destination}: {exc}"
        ) from exc
