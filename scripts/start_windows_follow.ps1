# Windows: start YOLO preview, SSH tunnel and Pi real follow with one command.
# Requires PC and Pi on the same reachable Wi-Fi, Pi power stable, and both repos updated.
param(
    [string]$PiHost = "pyiot014.local",
    [string]$PiUser = "pi"
)
$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$python = Join-Path $repo ".venv\Scripts\python.exe"
$gpuEntry = Join-Path $repo "experiments\pc_gpu\pc_detector_preview_server.py"
if (-not (Test-Path $python)) { throw "Windows Python venv missing: $python" }
if (-not (Test-Path $gpuEntry)) { throw "GPU server missing: $gpuEntry" }
if (-not (Get-Command ssh -ErrorAction SilentlyContinue)) {
    throw "OpenSSH client was not found."
}
$gpuProcess = $null
try {
    Write-Host "[1/3] Starting Windows RTX GPU YOLO preview (127.0.0.1:8765)"
    $gpuArg = ('"{0}" --device 0' -f $gpuEntry)
    $gpuProcess = Start-Process -FilePath $python -ArgumentList $gpuArg -WorkingDirectory $repo -PassThru
    Start-Sleep -Seconds 3
    $gpuProcess.Refresh()
    if ($gpuProcess.HasExited) {
        throw "GPU server exited. Check the Python/CUDA environment."
    }
    Write-Host "[2/3] Connecting secure SSH tunnel to $PiUser@$PiHost"
    Write-Host "[3/3] Starting Pi real follow in the same SSH session"
    Write-Host "When operator token appears, open http://127.0.0.1:8766/"
    Write-Host "Press Ctrl+C to stop the remote Pi process."
    $target = "$PiUser@$PiHost"
    $remote = "cd ~/PresentationEscort-demo && bash scripts/start_pi_follow.sh"
    $sshArgs = @(
        "-tt",
        "-o", "ExitOnForwardFailure=yes",
        "-o", "ServerAliveInterval=10",
        "-o", "ServerAliveCountMax=3",
        "-R", "127.0.0.1:8765:127.0.0.1:8765",
        "-L", "127.0.0.1:8766:127.0.0.1:8766",
        $target, $remote
    )
    & ssh @sshArgs
    if ($LASTEXITCODE -ne 0) {
        throw "SSH/Pi exited with code $LASTEXITCODE. Review Pi output."
    }
}
finally {
    if ($gpuProcess) {
        $gpuProcess.Refresh()
        if (-not $gpuProcess.HasExited) {
            Write-Host "Stopping the GPU process started by this launcher."
            Stop-Process -Id $gpuProcess.Id -ErrorAction SilentlyContinue
        }
    }
}
