"""Low-level SLMP boundary and pymcprotocol implementation."""

from __future__ import annotations

import logging
import math
from typing import Any, Optional, Protocol, Sequence

from .exceptions import SLMPCommunicationError


LOGGER = logging.getLogger("fx5u_controller.driver")


class SLMPDriver(Protocol):
    @property
    def connected(self) -> bool:
        ...

    def connect(self) -> None:
        ...

    def disconnect(self) -> None:
        ...

    def write_bits(self, head_device: str, values: Sequence[int]) -> None:
        ...

    def write_words(self, head_device: str, values: Sequence[int]) -> None:
        ...

    def read_bits(self, head_device: str, count: int) -> Sequence[int]:
        ...

    def read_words(self, head_device: str, count: int) -> Sequence[int]:
        ...


class PymcProtocolSLMPDriver:
    """SLMP 3E Binary adapter around ``pymcprotocol.Type3E``.

    pymcprotocol 0.3.0 has no iQ-F selector. Mitsubishi describes FX5U 3E as
    QnA-compatible; ``Q`` selects the required normal, non-iQ-R device format
    for the confirmed M and D commands. The selector remains configurable.
    """

    def __init__(
        self,
        ip: str,
        port: int,
        *,
        plc_type: str = "Q",
        socket_timeout_s: float = 2.0,
    ) -> None:
        if not isinstance(ip, str) or not ip.strip():
            raise ValueError("ip must be a non-empty IPv4 address or host name")
        if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65_535:
            raise ValueError("port must be an integer from 1 to 65535")
        if (
            isinstance(socket_timeout_s, bool)
            or not isinstance(socket_timeout_s, (int, float))
            or not math.isfinite(float(socket_timeout_s))
            or socket_timeout_s <= 0
        ):
            raise ValueError("socket_timeout_s must be a finite number greater than zero")

        self.ip = ip.strip()
        self.port = port
        self.plc_type = plc_type
        self.socket_timeout_s = float(socket_timeout_s)
        self._client: Optional[Any] = None
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        if self._connected:
            return
        try:
            import pymcprotocol
        except ImportError as exc:
            raise SLMPCommunicationError(
                "pymcprotocol is not installed; install the project dependencies first"
            ) from exc

        client: Optional[Any] = None
        try:
            client = pymcprotocol.Type3E(plctype=self.plc_type)
            client.setaccessopt(commtype="binary")
            client.soc_timeout = self.socket_timeout_s
            client.connect(self.ip, self.port)
        except Exception as exc:
            if client is not None:
                try:
                    client.close()
                except Exception:
                    pass
            LOGGER.error("SLMP connection failed endpoint=%s:%s error=%s", self.ip, self.port, exc)
            raise SLMPCommunicationError(
                f"Failed to connect to FX5U at {self.ip}:{self.port}: {exc}"
            ) from exc

        self._client = client
        self._connected = True
        LOGGER.info("SLMP connected endpoint=%s:%s frame=3E encoding=binary", self.ip, self.port)

    def disconnect(self) -> None:
        client = self._client
        self._client = None
        self._connected = False
        if client is None:
            return
        try:
            client.close()
        except Exception as exc:
            LOGGER.error("SLMP socket close failed error=%s", exc)
            raise SLMPCommunicationError(f"Failed to close the SLMP connection: {exc}") from exc
        LOGGER.info("SLMP disconnected endpoint=%s:%s", self.ip, self.port)

    def write_bits(self, head_device: str, values: Sequence[int]) -> None:
        data = list(values)
        LOGGER.debug("SLMP bit write head=%s values=%s", head_device, data)
        try:
            self._require_client().batchwrite_bitunits(headdevice=head_device, values=data)
        except Exception as exc:
            LOGGER.error(
                "SLMP bit write failed head=%s values=%s error=%s",
                head_device,
                data,
                exc,
            )
            raise SLMPCommunicationError(
                f"SLMP bit write failed at {head_device} for {data}: {exc}"
            ) from exc

    def write_words(self, head_device: str, values: Sequence[int]) -> None:
        data = list(values)
        LOGGER.debug("SLMP word write head=%s values=%s", head_device, data)
        try:
            # pymcprotocol uses signed=True here, preserving negative values.
            self._require_client().batchwrite_wordunits(headdevice=head_device, values=data)
        except Exception as exc:
            LOGGER.error(
                "SLMP word write failed head=%s values=%s error=%s",
                head_device,
                data,
                exc,
            )
            raise SLMPCommunicationError(
                f"SLMP word write failed at {head_device} for {data}: {exc}"
            ) from exc

    def read_bits(self, head_device: str, count: int) -> Sequence[int]:
        LOGGER.debug("SLMP bit read head=%s count=%s", head_device, count)
        try:
            values = self._require_client().batchread_bitunits(
                headdevice=head_device, readsize=count
            )
        except Exception as exc:
            LOGGER.error("SLMP bit read failed head=%s count=%s error=%s", head_device, count, exc)
            raise SLMPCommunicationError(
                f"SLMP bit read failed at {head_device} for {count} points: {exc}"
            ) from exc
        return self._validate_read_size(head_device, count, values)

    def read_words(self, head_device: str, count: int) -> Sequence[int]:
        LOGGER.debug("SLMP word read head=%s count=%s", head_device, count)
        try:
            values = self._require_client().batchread_wordunits(
                headdevice=head_device, readsize=count
            )
        except Exception as exc:
            LOGGER.error(
                "SLMP word read failed head=%s count=%s error=%s",
                head_device,
                count,
                exc,
            )
            raise SLMPCommunicationError(
                f"SLMP word read failed at {head_device} for {count} points: {exc}"
            ) from exc
        return self._validate_read_size(head_device, count, values)

    def _require_client(self) -> Any:
        if not self._connected or self._client is None:
            raise SLMPCommunicationError("SLMP driver is not connected")
        return self._client

    @staticmethod
    def _validate_read_size(head_device: str, count: int, values: Sequence[int]) -> Sequence[int]:
        data = list(values)
        if len(data) != count:
            raise SLMPCommunicationError(
                f"SLMP short response at {head_device}: expected {count}, received {len(data)}"
            )
        return data
