#!/usr/bin/env bash
# Pi real demo launcher. Password is entered only via NetworkManager.
set -Eeuo pipefail
PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SSID="obnizAP2"
SERIAL_PORT="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"

wifi_status() {
  if ! command -v nmcli >/dev/null 2>&1; then
    echo "NetworkManager (nmcli) is not installed." >&2
    return 1
  fi
  nmcli -t -f ACTIVE,SSID device wifi list --rescan no | grep '^yes:' || true
  hostname -I || true
}
MODE="run"
if (( $# > 0 )); then MODE="$1"; fi
if (( $# > 1 )); then echo "Unexpected extra arguments." >&2; exit 2; fi
case "$MODE" in
  --wifi-status)
    wifi_status
    exit 0
    ;;
  --setup-wifi)
    if ! command -v nmcli >/dev/null 2>&1; then
      echo "NetworkManager/nmcli is required." >&2; exit 2
    fi
    echo "Connecting to $SSID. SSH on the old Wi-Fi may disconnect."
    echo "NetworkManager will prompt for the password and remember it."
    if nmcli -t -f NAME connection show | grep -Fxq "$SSID"; then
      nmcli connection modify "$SSID" connection.autoconnect yes
      nmcli --wait 30 connection up id "$SSID"
    else
      nmcli --ask --wait 30 device wifi connect "$SSID"
      nmcli connection modify "$SSID" connection.autoconnect yes
    fi
    wifi_status
    exit 0
    ;;
  --help|-h)
    echo "Usage: bash scripts/start_pi_follow.sh [--setup-wifi|--wifi-status]"
    exit 0
    ;;
  run) ;;
  *) echo "Unknown argument: $MODE" >&2; exit 2 ;;
esac

# Neither Wi-Fi SSID nor historical voltage warnings block startup.
# Firmware ESTOP, watchdog, and actuator gates remain unchanged.
if command -v nmcli >/dev/null 2>&1; then
  if ! nmcli -t -f ACTIVE,SSID device wifi list --rescan no | grep -Fxq "yes:$SSID"; then
    echo "Warning: not on $SSID; continuing on the existing network." >&2
  fi
else
  echo "Warning: nmcli unavailable; cannot display current Wi-Fi." >&2
fi
if command -v vcgencmd >/dev/null 2>&1; then
  if POWER_STATE="$(vcgencmd get_throttled 2>/dev/null)" &&
      [[ "$POWER_STATE" =~ ^throttled=0x[0-9a-fA-F]+$ ]]; then
    POWER_HEX="$(printf '%s' "$POWER_STATE" | cut -d= -f2)"
    if (( POWER_HEX & 0x5 )); then
      echo "Warning: ACTIVE Pi undervoltage/throttling ($POWER_STATE)." >&2
      echo "Use a Pi-rated power supply; UART may fail. Continuing as requested." >&2
    fi
    if (( POWER_HEX & 0x50000 )); then
      echo "Warning: historical Pi undervoltage/throttling ($POWER_STATE)." >&2
    fi
  else
    echo "Warning: failed to read Pi voltage diagnostics; continuing." >&2
  fi
else
  echo "Warning: vcgencmd unavailable; continuing without voltage diagnostics." >&2
fi

if [[ ! -e "$SERIAL_PORT" ]]; then
  echo "ESP32 serial device missing: $SERIAL_PORT" >&2
  echo "Check: ls -l /dev/serial/by-id/" >&2
  exit 3
fi
PYTHON_BIN="python3"
if [[ -x "$PROJECT_ROOT/.venv/bin/python3" ]]; then
  PYTHON_BIN="$PROJECT_ROOT/.venv/bin/python3"
fi
cd "$PROJECT_ROOT"
echo "Wi-Fi: $SSID | Pi real ESP32 console starts in SAFE (not AUTO)."
echo "Open http://127.0.0.1:8766/ through the Windows SSH tunnel."
exec "$PYTHON_BIN" experiments/pc_gpu/pi_real_head_integration.py \
  --serial-port "$SERIAL_PORT" \
  --servo-power-on --pc-host 127.0.0.1 \
  --enable-head-test --enable-body-turns --body-turn-calibrated \
  --pitch-calibrated --pitch-min-deg -2 --pitch-max-deg 2 \
  --relative-pitch-reference --reverse-body-turn-direction \
  --distance-adaptive-head --responsive-follow --vision-verified-head \
  --camera-horizontal-fov-deg 65 --auto-recalibrate-brief-loss \
  --head-speed-deg-per-second 3.5 --pitch-speed-deg-per-second 2.5 \
  --turn-stable-frames 3 --turn-settle-seconds 0.7 --turn-edge-fraction 0.15 \
  --camera-width 640 --camera-height 480 --camera-fps 30 \
  --camera-mjpeg --jpeg-quality 70 --preview-every-n-frames 3 \
  --bench-secured --neutral-calibrated --physical-cutoff-tested \
  --capture-diagnostics
