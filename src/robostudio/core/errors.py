from enum import StrEnum


class ErrorCode(StrEnum):
    INVALID_MANIFEST = "INVALID_MANIFEST"
    UNSUPPORTED_VERSION = "UNSUPPORTED_VERSION"
    UNKNOWN_CAPABILITY = "UNKNOWN_CAPABILITY"
    DUPLICATE_ID = "DUPLICATE_ID"
    NOT_FOUND = "NOT_FOUND"
    INVALID_MAPPING = "INVALID_MAPPING"
    INVALID_DIMENSION = "INVALID_DIMENSION"
    FRAME_MISMATCH = "FRAME_MISMATCH"
    MODEL_MISMATCH = "MODEL_MISMATCH"
    INVALID_VALUE = "INVALID_VALUE"
    INVALID_LIFECYCLE = "INVALID_LIFECYCLE"
    UNSUPPORTED_CAPABILITY = "UNSUPPORTED_CAPABILITY"
    IO_ERROR = "IO_ERROR"
    INTEGRITY_ERROR = "INTEGRITY_ERROR"
    CONVERGENCE_FAILED = "CONVERGENCE_FAILED"
    TRACKING_FAILED = "TRACKING_FAILED"


class RoboStudioError(ValueError):
    def __init__(self, code: ErrorCode, message: str, path: str = "") -> None:
        self.code = code
        self.path = path
        self.message = message
        super().__init__(f"{code}: {path + ': ' if path else ''}{message}")

    def as_dict(self) -> dict[str, str]:
        return {"code": self.code.value, "path": self.path, "message": self.message}


def require(condition: bool, code: ErrorCode, message: str, path: str = "") -> None:
    if not condition:
        raise RoboStudioError(code, message, path)
