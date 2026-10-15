param([switch]$Force)
$ErrorActionPreference = 'Stop'
$projectRoot = $PSScriptRoot
$cadPython = Join-Path $projectRoot '.venv\Scripts\python.exe'
# v2: reuse the original Astra 01 environment (identical lock file) when this copy has none.
$siblingPython = Join-Path $projectRoot '..\drone_cad\.venv\Scripts\python.exe'
if (!(Test-Path -LiteralPath $cadPython) -and (Test-Path -LiteralPath $siblingPython)) { $cadPython = (Resolve-Path $siblingPython).Path }
if (!(Test-Path -LiteralPath $cadPython)) {
    $cadPython = Join-Path $projectRoot '.venv\Scripts\python.exe'
    & py -3.11 -m venv (Join-Path $projectRoot '.venv')
    if ($LASTEXITCODE -ne 0) { throw 'Python 3.11 is required. Install Python 3.11 and rerun.' }
    & $cadPython -m pip install -r (Join-Path $projectRoot 'requirements-lock.txt')
    if ($LASTEXITCODE -ne 0) { throw 'CAD dependency installation failed.' }
}
$buildArgs = @('-u', (Join-Path $projectRoot 'src\build.py'))
if ($Force) { $buildArgs += '--force' }
& $cadPython @buildArgs
if ($LASTEXITCODE -ne 0) { throw 'CAD build failed; inspect output/reports/error.txt and progress.log.' }
$junitPath = Join-Path $projectRoot 'output\reports\tests.xml'
& $cadPython -m pytest (Join-Path $projectRoot 'tests') -q "--junitxml=$junitPath"
$verificationExit = $LASTEXITCODE
& $cadPython (Join-Path $projectRoot 'src\package.py')
if ($LASTEXITCODE -ne 0) { throw 'Packaging failed.' }
if ($verificationExit -ne 0) { throw 'CAD verification failed; review packages include output/reports/validation.json and tests.xml. Do not treat these as validated manufacturing files.' }
Write-Host 'CAD outputs and validation are in' (Join-Path $projectRoot 'output')
