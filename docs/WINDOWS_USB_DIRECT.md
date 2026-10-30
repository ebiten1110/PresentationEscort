# Windows USB-direct offline demo

This optional path runs **entirely on Windows**. It does not use a Raspberry Pi,
Wi-Fi, internet, an SSH tunnel, or the remote GPU detection TCP protocol.

## Wiring (power off before changes)
- USB UVC camera -> Windows PC USB port (not Pi).
- ESP32 CH340 USB serial -> Windows PC USB port (usually COM8 on demo hardware).
- ESP32 -> PCA9685 and the existing LED wiring stay unchanged.
- Servos need their correct, independent power supply and an accessible
  physical power cutoff; USB alone must not be assumed to supply servo power.
- Secure robot against falling before any actuating test.
- Do not open the same ESP32 COM port with PlatformIO or another serial monitor
  at the same time.

## Preparation (while still online)
```powershell
cd "$HOME\PresentationEscort-demo"
git switch feature/pc-gpu-follow-2026-10-28
git pull --ff-only origin feature/pc-gpu-follow-2026-10-28
.\.venv\Scripts\python.exe -c "import cv2,torch,serial,ultralytics;print('CUDA',torch.cuda.is_available())"
.\.venv\Scripts\python.exe -m serial.tools.list_ports -v
Test-Path .\yolo26n-pose.pt
```

If the model is not in the repository root, find the already downloaded local
pose model and specify it using `-Model` (path to the existing .pt file).
**Do not assume models download automatically without internet access.**

## Monitor mode (default, never ARM motors)
```powershell
cd "$HOME\PresentationEscort-demo"
powershell -ExecutionPolicy Bypass -File .\scripts\start_windows_usb_follow.ps1 -EspPort COM8 -CameraIndex 0 -Mode Monitor
```
Open http://127.0.0.1:8766/ on the **Windows PC**, copy the displayed
Operator token into the console. Confirm video, detection boxes and FPS.
Monitor mode can read ESP32 telemetry but does not authorize head/body motion.
The gesture standby button is deliberately disabled.

Use `-CameraIndex 1` (or 2) if the computer's built-in camera is camera 0.
Use `-CameraBackend msmf` if DirectShow is incompatible with this USB camera.

## Actuating modes — require physical checks, never unattended
After confirming physical bench, neutral calibration, physical power cutoff,
servo V+ ON, and pitch calibration, head-only mode may be launched:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\start_windows_usb_follow.ps1 -EspPort COM8 -CameraIndex 0 -Mode Head -BenchSecured -NeutralCalibrated -PhysicalCutoffTested -ServoPowerOnConfirmed -PitchCalibrated
```

For body turning, also verify turn direction and mechanism physically, then
use `-Mode Body -BodyTurnCalibrated` with the same safety flags. Body mode still
blocks translation/forward walking. The standard gesture standby button can
then be enabled; holding both hands above the head requests lock and AUTO.

The flags are **operator declarations**, not software measurements; never
assert them without actual testing. On startup the system is SAFE, not AUTO.
The firmware watchdog, STOP/ESTOP, and the existing motion safety gateway
are still active; software STOP is NOT a physical cutoff.

## Monitoring and troubleshooting
- A black preview means verify USB camera permissions, index, USB power, or
  whether another app owns the webcam.
- `USB_CAMERA_UNAVAILABLE` means try other indexes or `-CameraBackend msmf`.
- Serial open errors mean verify COM8 and close PlatformIO/serial monitors.
- `model checkpoint not found locally` means pass an existing local model path.
- AUTO refused: check physical calibration, fresh head status and ESP32 state.
- The preview is intentionally throttled; the AI consumes every captured frame.
- Ctrl+C stops the local runner; physically cut servo power for storage/rewiring.

The existing `start_windows_follow.ps1` remains the original SSH/Pi version.
Do not invoke it for USB-direct offline use.
