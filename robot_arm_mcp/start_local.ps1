$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$serverPath = Join-Path $projectRoot "server.py"
$envPath = Join-Path $projectRoot ".env"

if (-not (Test-Path $envPath)) {
    Write-Error "Missing .env file at $envPath. Copy .env.example to .env first."
}

$pythonCommands = @(
    @{ Name = "python"; Command = { & python --version *> $null; "python" } },
    @{ Name = "py"; Command = { & py -3 --version *> $null; "py -3" } },
    @{ Name = "direct"; Command = {
        $candidate = "C:\Users\natej\AppData\Local\Programs\Python\Python313\python.exe"
        if (Test-Path $candidate) { $candidate } else { $null }
    } }
)

$resolved = $null
foreach ($entry in $pythonCommands) {
    try {
        $result = & $entry.Command
        if ($result) {
            $resolved = $result
            break
        }
    } catch {
        continue
    }
}

if (-not $resolved) {
    Write-Error "No working Python executable was found. Install Python or add it to PATH."
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
