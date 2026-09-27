<#
Starts armctl and, optionally, its Cloudflare tunnel.

    .\scripts\start.ps1              # local only
    .\scripts\start.ps1 -Sim         # simulated arm, no hardware
    .\scripts\start.ps1 -Public      # local + Cloudflare tunnel

Only one process can hold COM4. If the original stack is running, stop it
first or this will fail to open the serial port.
#>
[CmdletBinding()]
param(
    [switch]$Sim,
    [switch]$Public,
    [int]$Port = 7002
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot

$busy = Get-CimInstance Win32_Process -Filter "Name='python.exe'" |
        Where-Object { $_.CommandLine -like '*single_arm_server*' }
if ($busy -and -not $Sim) {
    Write-Warning "The original stack looks like it is running and holding COM4:"
    $busy | ForEach-Object { Write-Warning "  PID $($_.ProcessId)" }
    Write-Warning "Stop it first, or pass -Sim to run without hardware."
}

$serverArgs = @("run.py", "--port", $Port)
if ($Sim) { $serverArgs += "--sim" }

Write-Host "Starting armctl on port $Port ..." -ForegroundColor Cyan
$server = Start-Process -FilePath "python" -ArgumentList $serverArgs `
    -WorkingDirectory $root -PassThru

$healthy = $false
foreach ($attempt in 1..30) {
    Start-Sleep -Milliseconds 500
    try {
        $health = Invoke-RestMethod "http://127.0.0.1:$Port/healthz" -TimeoutSec 2
        Write-Host ("Loop {0:N1} Hz, {1:N2} ms/tick" -f $health.loop.actual_hz, $health.loop.tick_ms)
        $healthy = $true
        break
    } catch { }
}

if (-not $healthy) {
    Write-Error "armctl did not become healthy. Check the console window it opened."
    if (-not $server.HasExited) { Stop-Process -Id $server.Id -Force }
    exit 1
}

Write-Host "Dashboard: http://127.0.0.1:$Port" -ForegroundColor Green

if ($Public) {
    $configPath = Join-Path $root "cloudflare\cloudflared.yml"
    if ((Get-Content $configPath -Raw) -match "REPLACE_WITH_TUNNEL_UUID") {
        Write-Error "Fill in the tunnel UUID in cloudflare\cloudflared.yml first (see its header)."
        exit 1
    }
    Write-Host "Starting Cloudflare tunnel ..." -ForegroundColor Cyan
    Start-Process -FilePath "cloudflared" -ArgumentList @("tunnel", "--config", $configPath, "run")
    Write-Host "Public: see the hostname in cloudflare\coudflared.yml" -ForegroundColor Green
}
