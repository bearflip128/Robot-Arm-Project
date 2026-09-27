<#
Keeps armctl running on the real arm, detached from any terminal or app.

    powershell -NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File scripts\run_supervised.ps1

If run.py exits for any reason it is restarted after a short delay. That is
safe for hardware: armctl always starts disarmed and never enables torque on
its own. Output is appended to logs\armctl.out.log.

Stop it with scripts\stop.ps1, which ends this supervisor first so it cannot
restart the server behind you.
#>
[CmdletBinding()]
param(
    [int]$Port = 7002,
    [int]$RestartDelaySec = 5
)

$root = Split-Path -Parent $PSScriptRoot
$python = (Get-Command python -ErrorAction Stop).Source
$logDir = Join-Path $root "logs"
$log = Join-Path $logDir "armctl.out.log"
$pidFile = Join-Path $logDir "supervisor.pid"
New-Item -ItemType Directory -Force $logDir | Out-Null

# One supervisor at a time: a second one would fight the first for COM4.
if (Test-Path $pidFile) {
    $old = Get-Content $pidFile -ErrorAction SilentlyContinue
    if ($old -and (Get-Process -Id $old -ErrorAction SilentlyContinue)) {
        Write-Warning "Supervisor already running as PID $old."
        exit 1
    }
}
Set-Content -Path $pidFile -Value $PID -Encoding ascii

function Write-Log([string]$msg) {
    Add-Content -Path $log -Value ("{0:yyyy-MM-dd HH:mm:ss} supervisor  {1}" -f (Get-Date), $msg)
}

$env:PYTHONIOENCODING = "utf-8"
$env:PYTHONUNBUFFERED = "1"
try {
    while ($true) {
        Write-Log "starting run.py on port $Port"
        Push-Location $root
        try {
            & cmd.exe /c "`"$python`" run.py --port $Port >> `"$log`" 2>&1"
            $code = $LASTEXITCODE
        } finally {
            Pop-Location
        }
        Write-Log "run.py exited with code $code; restarting in $RestartDelaySec s"
        Start-Sleep -Seconds $RestartDelaySec
    }
} finally {
    Remove-Item $pidFile -ErrorAction SilentlyContinue
}
