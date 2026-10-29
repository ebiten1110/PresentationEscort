# Phase 27 — One-command real follow and obnizAP2 Wi-Fi (2026-10-29)

Two launchers run the existing Windows GPU -> SSH reverse/local tunnel -> Raspberry Pi camera and real ESP32 setup. No password, auth token, or AUTO command is stored in the repository.

## Three operational steps

1. **Power and Pi Wi-Fi (once)**: Replace Raspberry Pi power from the PC USB port with a Pi-rated independent stable supply; previous kernel logs showed frequent undervoltage. On Pi, run: bash scripts/start_pi_follow.sh --setup-wifi. NetworkManager prompts for the Wi-Fi password without writing it in the terminal command or GitHub, saves it for automatic reconnection, and may disconnect an SSH session on the previous Wi-Fi. Connect the Windows PC to the same SSID obnizAP2, with client-to-client traffic permitted.
2. **Update both repository copies**: Before switching to an AP that may lack Internet, run git pull --ff-only origin feature/pc-gpu-follow-2026-10-28 in both Windows and Pi PresentationEscort-demo repositories. To learn the Pi IPv4, run bash scripts/start_pi_follow.sh --wifi-status on Pi. The Windows copy needs the existing .venv and GPU inference dependencies.
3. **Start from Windows PowerShell**: In the repository, run: powershell -ExecutionPolicy Bypass -File .\scripts\start_windows_follow.ps1 -PiHost <Pi_IP>. It starts the GPU preview process, opens SSH with both a loopback 8765 reverse forward and a loopback 8766 local forward, and runs the Pi script in that same session. When the Pi reports an operator token, open http://127.0.0.1:8766/ in Windows and enter that token. Pi starts in SAFE without automatic AUTO/ARM; the operator still enables gesture standby. Ctrl+C ends the session.

Alternative: bash scripts/start_pi_follow.sh runs the Pi side alone when PC YOLO and SSH tunnel are already up.

## Guards and behavior

- The Pi start script requires an active obnizAP2 Wi-Fi connection; it never switches Wi-Fi during an SSH-driven motor session. Initial setup uses nmcli --ask, and NetworkManager retains credentials locally.
- The script refuses to open the ESP32 serial port when vcgencmd get_throttled reports *current* undervoltage or throttling; historical flags emit a warning instead. It checks the stable by-id serial symlink before starting.
- The previous externally calibrated yaw/pitch/body options, distance-adaptive tracking, preview decimation and diagnostic capture are preserved. Firmware is unchanged. Physical cutoff remains necessary; STOP/ESTOP alone is not physical power removal.
- The Windows PC and Pi must reach each other over the AP. If the AP enables client isolation, SSH will not connect even with matching SSIDs. Use -PiHost with the Pi's current IPv4 when .local does not resolve.
