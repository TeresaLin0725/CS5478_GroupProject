param(
    [ValidateSet('nearest', 'llm_guided', 'both')]
    [string]$Policy = 'nearest'
)
$ErrorActionPreference = 'Stop'
$projectPath = Split-Path -Parent $PSScriptRoot
$pythonPath = Join-Path $projectPath 'data/venv-igibson/Scripts/python.exe'
if (-not (Test-Path -LiteralPath $pythonPath)) {
    throw 'Build iGibson first: python scripts/setup_igibson.py (see Documentation/chinese/igibson_demo.md).'
}
$env:PYTHONPATH = Join-Path $projectPath 'src'
$env:PYTHONIOENCODING = 'utf-8'
$env:PYTHONWARNINGS = 'ignore'
$configPath = Join-Path $projectPath '.env'
if (-not (Test-Path -LiteralPath $configPath)) {
    $configPath = Join-Path (Split-Path -Parent $projectPath) '.env'
}
Push-Location $projectPath
try {
    # iGibson's gym import prints a deprecation banner on stderr; PowerShell 5.1
    # would turn that into a terminating error under 'Stop'.
    $ErrorActionPreference = 'Continue'
    & $pythonPath -m frontier_search.runner.igibson_demo --policy $Policy --env-file $configPath
    $demoExitCode = $LASTEXITCODE
} finally {
    Pop-Location
}
exit $demoExitCode
