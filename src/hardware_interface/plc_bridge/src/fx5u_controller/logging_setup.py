"""Application logging with a rotating on-disk black-box log."""

from __future__ import annotations

import logging
import time
from logging.handlers import RotatingFileHandler

from .models import LoggingConfig


LOGGER_NAME = "fx5u_controller"


class UTCFormatter(logging.Formatter):
    converter = time.gmtime


def configure_logging(config: LoggingConfig) -> logging.Logger:
    config.file.parent.mkdir(parents=True, exist_ok=True)
    logger = logging.getLogger(LOGGER_NAME)
    logger.setLevel(getattr(logging, config.level))
    logger.propagate = False

    for handler in list(logger.handlers):
        if getattr(handler, "_fx5u_managed", False):
            logger.removeHandler(handler)
            handler.close()

    formatter = UTCFormatter(
        fmt="%(asctime)s.%(msecs)03dZ %(levelname)s %(threadName)s "
        "%(name)s %(message)s",
        datefmt="%Y-%m-%dT%H:%M:%S",
    )
    file_handler = RotatingFileHandler(
        config.file,
        maxBytes=config.max_bytes,
        backupCount=config.backup_count,
        encoding="utf-8",
    )
    file_handler.setFormatter(formatter)
    file_handler.setLevel(getattr(logging, config.level))
    file_handler._fx5u_managed = True  # type: ignore[attr-defined]
    logger.addHandler(file_handler)

    if config.console:
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        console_handler.setLevel(getattr(logging, config.level))
        console_handler._fx5u_managed = True  # type: ignore[attr-defined]
        logger.addHandler(console_handler)

    logger.info(
        "logging initialized file=%s max_bytes=%s backup_count=%s",
        config.file,
        config.max_bytes,
        config.backup_count,
    )
    return logger


def shutdown_logging() -> None:
    """Flush, close, and remove handlers installed by ``configure_logging``."""

    logger = logging.getLogger(LOGGER_NAME)
    for handler in list(logger.handlers):
        if getattr(handler, "_fx5u_managed", False):
            logger.removeHandler(handler)
            handler.flush()
            handler.close()
