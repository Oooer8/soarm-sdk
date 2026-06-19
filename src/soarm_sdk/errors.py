class SOARMError(Exception):
    """Base exception for SOARM SDK errors."""


class ConfigurationError(SOARMError):
    """Raised when the SDK configuration is invalid."""


class HardwareError(SOARMError):
    """Raised when communication with the arm hardware fails."""


class NotConnectedError(HardwareError):
    """Raised when a hardware operation requires an active connection."""


class SafetyError(SOARMError):
    """Raised when a safety check fails."""


class LimitViolation(SafetyError):
    """Raised when a target exceeds configured limits."""


class EmergencyStopActive(SafetyError):
    """Raised when a motion command is issued while emergency stop is active."""


class MotionError(SOARMError):
    """Raised when a motion command cannot be executed."""


class CalibrationError(SOARMError):
    """Raised when calibration cannot be completed."""


class UnsupportedFeature(SOARMError):
    """Raised when an unsupported operation is requested."""
