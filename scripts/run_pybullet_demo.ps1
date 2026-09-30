param(
    [ValidateSet('nearest', 'llm_guided', 'both')]
    [string]$Policy = 'nearest',
    [switch]$Gui,
    [switch]$Blocked
)
$ErrorActionPreference = 'Stop'
$projectPath = Split-Path -Parent $PSScriptRoot
$pythonPath = Join-Path $projectPath '.venv/Scripts/python.exe'
if (-not (Test-Path -LiteralPath $pythonPath)) {
    throw 'Create .venv and install requirements-pybullet.txt first. See README.md.'
}
$env:PYTHONPATH = Join-Path $projectPath 'src'
$env:PYTHONIOENCODING = 'utf-8'
$configPath = Join-Path $projectPath '.env'
if (-not (Test-Path -LiteralPath $configPath)) {
    $configPath = Join-Path (Split-Path -Parent $projectPath) '.env'
}
$demoArguments = @('-m', 'frontier_search.runner.pybullet_demo', '--policy', $Policy, '--env-file', $configPath)
if ($Gui) { $demoArguments += '--gui' }
if ($Blocked) { $demoArguments += '--blocked' }
Push-Location $projectPath
try {
    & $pythonPath @demoArguments
    $demoExitCode = $LASTEXITCODE
} finally {
    Pop-Location
}
exit $demoExitCode
