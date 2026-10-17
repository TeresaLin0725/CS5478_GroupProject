param(
    [ValidateSet('random','nearest','information_gain','llm_guided','both','all')]
    [string]$Policy = 'all',
    [ValidateSet('apartment','clutter','dynamic','suite')]
    [string]$Scenario = 'suite',
    [ValidateSet('replay','live')]
    [string]$Backend = 'replay',
    [string]$Output = ''
)
$ErrorActionPreference = 'Stop'
$projectPath = Split-Path -Parent $PSScriptRoot
$pythonPath = Join-Path $projectPath 'data/venv-igibson/Scripts/python.exe'
if (-not (Test-Path -LiteralPath $pythonPath)) {
    throw 'Run python integration/scripts/setup_igibson.py first to build iGibson and download Rs/Locobot assets.'
}
if (-not $Output) { $Output = 'runs/igibson-realistic-' + (Get-Date -Format 'yyyyMMdd-HHmmss') }
if (-not [IO.Path]::IsPathRooted($Output)) { $Output = Join-Path $projectPath $Output }
$outputPath = [IO.Path]::GetFullPath($Output)
if (Test-Path -LiteralPath $outputPath) { throw 'Choose a fresh output directory.' }
$env:PYTHONPATH = Join-Path $projectPath 'src'
$env:PYTHONIOENCODING = 'utf-8'
$configPath = Join-Path $projectPath '.env'
if (-not (Test-Path -LiteralPath $configPath)) { $configPath = Join-Path (Split-Path -Parent $projectPath) '.env' }
Push-Location $projectPath
try {
    $ErrorActionPreference = 'Continue'
    & $pythonPath -m frontier_search.runner.igibson_demo --policy $Policy --scenario $Scenario --backend $Backend --env-file $configPath --output $outputPath
    $runCode = $LASTEXITCODE
    Write-Host ('Open: ' + (Join-Path $outputPath 'index.html'))
} finally { Pop-Location }
exit $runCode
