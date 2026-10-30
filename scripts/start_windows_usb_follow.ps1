# PresentationEscort: Windows USB-direct launch, no Pi/SSH/Wi-Fi.
# Starts SAFE / no actuation in Monitor mode by default.
param(
    [string]$EspPort = "COM8",
    [int]$CameraIndex = 0,
    [ValidateSet("Monitor", "Head", "Body")]
    [string]$Mode = "Monitor",
    [ValidateSet("auto", "dshow", "msmf")]
    [string]$CameraBackend = "auto",
    [string]$Model = "yolo26n-pose.pt",
    [int]$ImageSize = 640,
    [ValidateRange(1,10)][int]$PreviewEvery = 2,
    [switch]$BenchSecured,
    [switch]$NeutralCalibrated,
    [switch]$PhysicalCutoffTested,
    [switch]$ServoPowerOnConfirmed,
    [switch]$PitchCalibrated,
    [switch]$BodyTurnCalibrated,
    [switch]$CaptureDiagnostics
)

$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$python = Join-Path $repo ".venv\Scripts\python.exe"
$entry = Join-Path $repo "experiments\pc_gpu\windows_usb_follow.py"
if (-not (Test-Path $python)) { throw "Python virtual environment missing: $python" }
if (-not (Test-Path $entry)) { throw "Windows direct runner missing: $entry" }

if ($Mode -ne "Monitor" -and -not (
    $BenchSecured -and $NeutralCalibrated -and $PhysicalCutoffTested -and
    $ServoPowerOnConfirmed -and $PitchCalibrated
)) {
    throw "Head/Body requires ALL: -BenchSecured -NeutralCalibrated -PhysicalCutoffTested -ServoPowerOnConfirmed -PitchCalibrated. Verify physically first."
}
if ($Mode -eq "Body" -and -not $BodyTurnCalibrated) {
    throw "Body mode requires -BodyTurnCalibrated and actual physical verification."
}

$pyArgs = @(
    $entry,
    "--serial-port", $EspPort,
    "--camera-index", [string]$CameraIndex,
    "--camera-backend", $CameraBackend,
    "--image-size", [string]$ImageSize,
    "--preview-every", [string]$PreviewEvery,
    "--mode", $Mode.ToLowerInvariant(),
    "--model", $Model
)
if ($BenchSecured) { $pyArgs += "--bench-secured" }
if ($NeutralCalibrated) { $pyArgs += "--neutral-calibrated" }
if ($PhysicalCutoffTested) { $pyArgs += "--physical-cutoff-tested" }
if ($ServoPowerOnConfirmed) { $pyArgs += "--servo-power-on-confirmed" }
if ($PitchCalibrated) { $pyArgs += "--pitch-calibrated" }
if ($BodyTurnCalibrated) { $pyArgs += "--body-turn-calibrated" }
if ($CaptureDiagnostics) { $pyArgs += "--capture-diagnostics" }

Write-Host "[Windows USB Direct] Wi-Fi: not required | SSH: not required"
Write-Host "Camera: index $CameraIndex, backend $CameraBackend | ESP32: $EspPort | mode: $Mode"
Write-Host "Open http://127.0.0.1:8766/ and enter the printed Operator token."
Write-Host "Press Ctrl+C to stop. STOP is not a physical power cut."
Push-Location $repo
try {
    & $python @pyArgs
    if ($LASTEXITCODE -ne 0) {
        throw "USB Direct runner exited with code $LASTEXITCODE"
    }
}
finally {
    Pop-Location
}
