from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from fx5u_controller import configure_logging, shutdown_logging
from fx5u_controller.models import LoggingConfig


class LoggingTests(unittest.TestCase):
    def tearDown(self) -> None:
        shutdown_logging()

    def test_rotating_file_log_is_created_and_rotated(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            log_path = Path(directory) / "black-box.log"
            logger = configure_logging(
                LoggingConfig(
                    level="DEBUG",
                    file=log_path,
                    max_bytes=200,
                    backup_count=2,
                    console=False,
                )
            )
            for index in range(30):
                logger.info("test command index=%s values=%s", index, [1, 2, 3, 4])
            for handler in logger.handlers:
                handler.flush()

            self.assertTrue(log_path.exists())
            self.assertTrue((Path(str(log_path) + ".1")).exists())


if __name__ == "__main__":
    unittest.main()

