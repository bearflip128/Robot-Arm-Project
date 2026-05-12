$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$serverPath = Join-Path $projectRoot "app\single_arm_server.py"
$envPath = Join-Path $projectRoot ".env"

if (-not (Test-Path $envPath)) {
    Write-Error "Missing .env file at $envPath."
}

$pythonCandidates = @(
    "python",
    "py -3",
    "C:\Users\natej\AppData\Local\Programs\Python\Python313\python.exe"
)

$resolved = $null
foreach ($candidate in $pythonCandidates) {
    try {
        if ($candidate -eq "python") {
            & python --version *> $null
            $resolved = $candidate
            break
        }
        if ($candidate -eq "py -3") {
            & py -3 --version *> $null
            $resolved = $candidate
            break
        }
        if (Test-Path $candidate) {
            $resolved = $candidate
            break
        }
    } catch {
        continue
    }
}

if (-not $resolved) {
    Write-Error "No working Python executable was found."
}

Push-Location $projectRoot
try {
    if ($resolved -eq "python") {
        & python $serverPath
    } elseif ($resolved -eq "py -3") {
        & py -3 $serverPath
    } else {
        & $resolved $serverPath
    }
} finally {
    Pop-Location
}
