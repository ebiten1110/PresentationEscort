"""Pi-to-ESP32 STOP-only connection probe (NOT a movement test).

Default = virtual serial sink, and never touches the ESP32.
--enable-serial MUST be supplied explicitly to open a real serial port.
Never use until the servo external supply is OFF and wiring is confirmed.
"""
from __future__ import annotations

import argparse
import time

from motion_gateway import MemoryCommandSink, PiMotionGateway
from serial_command_sink import SerialCommandSink


def main() -> None:
    parser = argparse.ArgumentParser(description="SAFE-only Pi serial probe")
    parser.add_argument("--enable-serial", action="store_true")
    parser.add_argument("--port", default=None, help="e.g. /dev/serial/by-id/...")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--reset-wait", type=float, default=2.5)
    args = parser.parse_args()

    if args.enable_serial:
        if not args.port:
            parser.error("--port is required with --enable-serial")
        if args.reset_wait < 2:
            parser.error("--reset-wait must be at least 2 seconds")
        print("[WARNING] Real ESP32 serial port will open (may reset ESP32).")
        sink = SerialCommandSink(args.port, baudrate=args.baud)
        time.sleep(args.reset_wait)  # Allow USB-triggered ESP32 reboot.
    else:
        print("[DRY-RUN] No serial port opened. STOP only in memory.")
        sink = MemoryCommandSink()

    try:
        with PiMotionGateway(sink) as gateway:
            print("mode=", gateway.status().safety.mode.value)
            print("last_command=", gateway.status().last_command)
            print("No movement command or AUTO ARM is supported by this tool.")
    finally:
        # PiMotionGateway owns and closes the sink in __exit__.
        # No further hardware action here.
        pass


if __name__ == "__main__":
    main()
