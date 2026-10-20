"""Phase 8 staged Raspberry Pi launcher — NEVER enables physical motion.

DEFAULT: offline compatibility audit, no camera/serial/network.
CONSOLE: Phase 7 dry-run dashboard; cannot send ESP32 commands.
DIAGNOSTIC: explicitly opted-in USB serial version probe. Opening USB may
reset the ESP32 and initialize servo outputs; external servo power MUST be
disconnected before using this mode.

Real hardware AUTO/MANUAL are intentionally NOT exposed until on-device
safety checks, independent physical stop, and Pi/ESP32 integration tests pass.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Optional

from firmware_preflight import FirmwarePreflight


ROOT = Path(__file__).resolve().parents[2]


def repository_audit(root: Path = ROOT) -> dict:
    checks = {
        "watchdog_750ms": (root / "esp32/src/safety_watchdog.h", "TIMEOUT_MS = 750U"),
        "firmware_protocol_query": (root / "esp32/src/safety_runtime.cpp", 'command == "SAFETY_VERSION"'),
        "firmware_starts_disarmed": (root / "esp32/src/safety_watchdog.h", "state_(State::DISARMED)"),
        "pi_requires_firmware_ack": (root / "experiments/pc_gpu/serial_command_sink.py", "requires_firmware_safety = True"),
        "safe_control_gateway": (root / "experiments/pc_gpu/motion_gateway.py", "class PiMotionGateway"),
        "offline_dashboard": (root / "experiments/pc_gpu/pi_operator_console.py", "MemoryCommandSink()"),
        "hardware_stop": (root / "esp32/src/safety_runtime.cpp", "stopWalking();"),
    }
    status = {}
    for key, (path, needle) in checks.items():
        try:
            status[key] = needle in path.read_text(encoding="utf-8")
        except OSError:
            status[key] = False
    return status


def diagnostic_probe(
    sink,
    *,
    reset_wait_s: float = 2.5,
    response_timeout_s: float = 3.0,
) -> tuple[bool, str]:
    """Probe only. Never transmits ARM, PING, RESET or motion.

    Caller MUST have physically disconnected servo V+ beforehand.
    sink is already opened exclusively by the caller.
    """
    if reset_wait_s < 2.0 or response_timeout_s < 0.5:
        raise ValueError("unsafe diagnostic timing")
    time.sleep(reset_wait_s)  # USB DTR may reboot the ESP32.
    probe = FirmwarePreflight()
    sink.send("SAFETY_VERSION")
    sink.send("SAFETY_STATUS")
    deadline = time.monotonic() + response_timeout_s
    while time.monotonic() < deadline:
        part = sink.read_available(max_bytes=4096)
        if part:
            probe.feed(part)
            state = probe.status()
            if state.reason in ("PROTOCOL_V1_DISARMED", "FIRMWARE_NOT_DISARMED",
                                "FIRMWARE_PROTOCOL_MISMATCH"):
                return state.safe_for_diagnostics, state.reason
        time.sleep(0.03)
    state = probe.status()
    return False, state.reason


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Staged Presentation Escort deployment — no motor outputs"
    )
    parser.add_argument(
        "--stage", default="audit",
        choices=("audit", "console", "diagnostic"),
        help="audit is the safe, no-device default",
    )
    parser.add_argument("--pc-host", default=None, help="GPU PC IP (console only)")
    parser.add_argument("--console-port", type=int, default=8766)
    parser.add_argument("--serial-port", default=None, help="diagnostic only, e.g. /dev/ttyUSB0")
    parser.add_argument("--servo-power-off-confirmed", action="store_true")
    return parser.parse_args(argv)


def run(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    audit = repository_audit()
    for key, passed in audit.items():
        print(f"[{'OK' if passed else 'FAIL'}] {key}", flush=True)
    if not all(audit.values()):
        print("Repository safety compatibility check FAILED. No device opened.")
        return 2

    if args.stage == "audit":
        print("Offline audit complete. No camera, serial port, or motors opened.")
        return 0

    if args.stage == "console":
        if args.serial_port or args.servo_power_off_confirmed:
            print("Serial flags are forbidden in dry-run console mode.")
            return 2
        from pi_operator_console import main as run_console
        console_args = ["--port", str(args.console_port)]
        if args.pc_host:
            console_args.extend(("--pc-host", args.pc_host))
        print("Launching memory-only operator console. Motor output DISABLED.")
        run_console(console_args)
        return 0

    if args.pc_host:
        print("GPU host cannot be combined with diagnostics.")
        return 2
    if not args.serial_port or not args.servo_power_off_confirmed:
        print("Diagnostic requires --serial-port and --servo-power-off-confirmed.")
        return 2

    # Serial opening can reset the ESP32, sending startup servo position
    # commands. The user must turn off external servo power BEFORE this point.
    from serial_command_sink import SerialCommandSink
    print("Diagnostic-only: checking firmware V1 and DISARMED, no motor commands.")
    try:
        with_serial = SerialCommandSink(args.serial_port)
        try:
            ok, reason = diagnostic_probe(with_serial)
        finally:
            with_serial.close()
    except (OSError, ImportError, ValueError) as exc:
        print("Diagnostic FAILED:", type(exc).__name__)
        return 2
    print(("PASS: " if ok else "BLOCKED: ") + reason)
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(run())
