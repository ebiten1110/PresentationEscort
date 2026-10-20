"""Strict firmware-version/status compatibility check before physical deployment.

Software-only parser: does not open ports or issue motion commands. A match
means the serial firmware speaks V1 and reports DISARMED; it does NOT prove
wiring, servo safety, or real STOP timing.
"""
from __future__ import annotations

import re
from dataclasses import dataclass

_EXPECTED_VERSION = "[Safety] Protocol=V1 TimeoutMs=750"
_STATE = re.compile(r"^\[Safety\] State: (ARMED|DISARMED|LATCHED)$")


@dataclass(frozen=True)
class PreflightStatus:
    compatible: bool
    disarmed: bool
    safe_for_diagnostics: bool
    reason: str


class FirmwarePreflight:
    """Consume bounded UART text (possibly arriving in several chunks)."""

    def __init__(self) -> None:
        self._pending = ""
        self._version_seen = False
        self._state = "UNKNOWN"
        self._version_conflict = False

    def feed(self, data: bytes) -> None:
        combined = self._pending + data.decode("utf-8", errors="replace")
        lines = combined.split("\n")
        self._pending = lines.pop()[-256:]
        for raw in lines:
            line = raw.strip()
            if line == _EXPECTED_VERSION:
                self._version_seen = True
            elif line.startswith("[Safety] Protocol="):
                self._version_conflict = True
            else:
                match = _STATE.fullmatch(line)
                if match:
                    self._state = match.group(1)

    def status(self) -> PreflightStatus:
        if self._version_conflict:
            return PreflightStatus(False, False, False, "FIRMWARE_PROTOCOL_MISMATCH")
        if not self._version_seen:
            return PreflightStatus(False, False, False, "FIRMWARE_VERSION_UNCONFIRMED")
        if self._state == "UNKNOWN":
            # UART replies may arrive in separate packets. Do not give up
            # before the full safety status has been received.
            return PreflightStatus(True, False, False, "FIRMWARE_STATE_UNCONFIRMED")
        if self._state != "DISARMED":
            return PreflightStatus(True, False, False, "FIRMWARE_NOT_DISARMED")
        return PreflightStatus(True, True, True, "PROTOCOL_V1_DISARMED")
