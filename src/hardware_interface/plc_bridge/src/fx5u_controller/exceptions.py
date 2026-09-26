"""Domain exceptions for the FX5U controller package."""


class WheelControllerError(RuntimeError):
    """Base exception for controller failures."""


class ConfigurationError(WheelControllerError):
    """Raised when a deployment configuration is missing or invalid."""


class ControllerStateError(WheelControllerError):
    """Raised when an operation is unsafe in the current software state."""


class SLMPCommunicationError(WheelControllerError):
    """Raised when an SLMP connection or command fails."""


class PulseCleanupError(SLMPCommunicationError):
    """Raised when a pulse failed and its OFF cleanup also failed."""

    def __init__(
        self,
        message: str,
        primary_error: BaseException,
        cleanup_error: BaseException,
    ) -> None:
        super().__init__(message)
        self.primary_error = primary_error
        self.cleanup_error = cleanup_error

