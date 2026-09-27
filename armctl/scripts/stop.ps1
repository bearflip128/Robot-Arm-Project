<#
Stops the supervised armctl server (see run_supervised.ps1).
The supervisor is ended first so it cannot restart run.py.
#>
$root = Split-Path -Parent $PSScriptRoot
$pidFile = Join-Path $root "logs\supervisor.pid"

if (Test-Path $pidFile) {
    $sup = Get-Content $pidFile -ErrorAction SilentlyContinue
    if ($sup) { Stop-Process -Id $sup -Force -ErrorAction SilentlyContinue }
    Remove-Item $pidFile -ErrorAction SilentlyContinue
}

Get-CimInstance Win32_Process -Filter "Name='python.exe'" |
    Where-Object { $_.CommandLine -like '* run.py --port *' } |
    ForEach-Object {
        Write-Host "Stopping armctl PID $($_.ProcessId)"
        Stop-Process -Id $_.ProcessId -Force
    }
