"""Robot-side Modbus adapter. Sequence numbers advance only after a real acknowledgement."""

from __future__ import annotations

import socket
import struct
import time
from dataclasses import dataclass

from advaitic_systems.modbus import PLCClient


def command_frame(transaction: int, sequence: int, station: int, kit_id: int, command: int) -> bytes:
    pdu = struct.pack(">BHHB4H", 16, 100, 4, 8, sequence, station, kit_id, command)
    return struct.pack(">HHHB", transaction, 0, len(pdu) + 1, 1) + pdu


@dataclass
class CommandResult:
    sequence: int
    station: int
    kit_id: int
    command: int
    duplicated: bool
    first_hex: str | None = None
    retry_hex: str | None = None
    accepted_before: int | None = None
    accepted_after: int | None = None
    applied_count: int = 1


class RobotPLC:
    def __init__(self, host: str = "127.0.0.1", port: int = 1502):
        self.host = host
        self.port = port
        self.client = PLCClient(host=host, port=port)
        self.acked = 0
        self._tx = 10

    def _next_tx(self) -> int:
        self._tx = (self._tx + 1) % 65536
        return self._tx

    def read(self, start: int, count: int = 1) -> list[int]:
        return self.client.read(start, count)

    def accepted(self, station: int) -> int:
        address = 14 if station == 1 else 24
        return int(self.read(address, 1)[0])

    def latest_kit(self, station: int) -> int:
        address = 11 if station == 1 else 21
        return int(self.read(address, 1)[0])

    def ready_flag(self, station: int) -> int:
        address = 10 if station == 1 else 20
        return int(self.read(address, 1)[0])

    def command(self, station: int, kit_id: int, command: int, drop_ack: bool = False) -> CommandResult:
        sequence = self.acked + 1
        if not drop_ack:
            self.client.command(sequence, station, kit_id, command)
            self.acked = sequence
            return CommandResult(sequence, station, kit_id, command, duplicated=False)
        return self._drop_then_retry(sequence, station, kit_id, command)

    def _drop_then_retry(self, sequence: int, station: int, kit_id: int, command: int) -> CommandResult:
        """Apply a write, close before the response, then retry the same payload."""
        first = command_frame(self._next_tx(), sequence, station, kit_id, command)
        self._send_and_close(first)
        if not self._wait_sequence(sequence):
            # The socket died before the PLC read the request. Send-and-close once more
            # so the documented fault is "ack lost after apply", not a lost request.
            first = command_frame(self._next_tx(), sequence, station, kit_id, command)
            self._send_and_close(first)
            if not self._wait_sequence(sequence):
                raise TimeoutError("PLC did not apply the command before the acknowledgement was dropped")
        accepted_address = 14 if station == 1 else 24
        accepted_before = int(self.read(accepted_address, 1)[0])
        retry = command_frame(self._next_tx(), sequence, station, kit_id, command)
        # A fresh client reads the response. The PLC must treat this as a duplicate.
        PLCClient(host=self.host, port=self.port).command(sequence, station, kit_id, command)
        status = int(self.read(2, 1)[0])
        accepted_after = int(self.read(accepted_address, 1)[0])
        if status != 2:
            raise RuntimeError(f"retry was not acknowledged as a duplicate (status {status})")
        self.acked = sequence
        return CommandResult(
            sequence, station, kit_id, command, duplicated=True,
            first_hex=first.hex(), retry_hex=retry.hex(),
            accepted_before=accepted_before, accepted_after=accepted_after, applied_count=1,
        )

    def _send_and_close(self, frame: bytes) -> None:
        with socket.create_connection((self.host, self.port), timeout=2) as stream:
            stream.sendall(frame)

    def _wait_sequence(self, sequence: int, timeout: float = 2.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                if int(self.read(1, 1)[0]) == sequence:
                    return True
            except (RuntimeError, OSError, TimeoutError):
                pass
            time.sleep(0.01)
        return False
