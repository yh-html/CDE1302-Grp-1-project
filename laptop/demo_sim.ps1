<#
.SYNOPSIS
  One-click demo of the tag-search mission in the built-in simulator.

.DESCRIPTION
  1. Runs run_tag_mission.py --simulate (timestamped events, tag table, error table).
  2. Draws the run with analyze_flight.py (search area, drone path, tag estimates vs
     reference positions) and opens the picture.

  The reference positions come from the simulation.tags block of tag_mission.yaml.
  This is a software simulation of the mission logic. It uses the same camera geometry
  as the code that localises the tags, so its tiny errors do NOT show real-world accuracy.

.PARAMETER DroneId
  Drone ID passed to the mission (default 22).

.PARAMETER NoOpen
  Do not open the picture afterwards.

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File laptop\demo_sim.ps1
#>
param(
    [int]$DroneId = 22,
    [switch]$NoOpen
)

$ErrorActionPreference = 'Stop'
$laptop = $PSScriptRoot
$root   = Split-Path $laptop -Parent

# Python: the project's conda env if it exists, otherwise whatever 'python' is on the PATH.
$py = Join-Path $root 'envs\cde1302-laptop\python.exe'
if (-not (Test-Path $py)) {
    $found = Get-Command python -ErrorAction SilentlyContinue
    if (-not $found) { throw "No Python found. Install Python 3.10+ and run: python -m pip install -r laptop\requirements.txt" }
    $py = $found.Source
}
Write-Host "Python: $py"

$checkPy = "import importlib.util as u; print(' '.join(m for m in ('yaml','numpy','matplotlib','pyulog','scipy') if u.find_spec(m) is None))"
$missing = (& $py -c $checkPy).Trim()
if ($missing) {
    Write-Host "Missing Python packages: $missing" -ForegroundColor Yellow
    Write-Host "Install them with:  `"$py`" -m pip install -r `"$laptop\requirements.txt`"" -ForegroundColor Yellow
    throw "Cannot continue until the packages are installed."
}

# Reference tag positions used by the simulator, read from tag_mission.yaml.
$yaml    = Join-Path $laptop 'tag_mission.yaml'
$truthPy = "import yaml; d=yaml.safe_load(open(r'$yaml')); t=d['simulation']['tags']; print(' '.join('%s:%s,%s' % (k,v[0],v[1]) for k,v in t.items()))"
$truth   = (& $py -c $truthPy).Trim()
if (-not $truth) { throw "No simulation.tags found in $yaml" }
Write-Host "Reference tags: $truth"
Write-Host ""

Write-Host "=== 1. Simulated mission ===" -ForegroundColor Cyan
& $py (Join-Path $laptop 'run_tag_mission.py') --drone-id $DroneId --simulate --truth $truth
if ($LASTEXITCODE -ne 0) { throw "run_tag_mission.py failed with exit code $LASTEXITCODE" }

$logs = Join-Path $laptop 'mission_logs'
$run  = Get-ChildItem $logs -Directory -Filter "*_drone${DroneId}_sim" |
        Sort-Object LastWriteTime -Descending | Select-Object -First 1
if (-not $run) { throw "No simulation run folder found in $logs" }

Write-Host ""
Write-Host "=== 2. Picture of the run ===" -ForegroundColor Cyan
$truthArgs = @()
foreach ($t in $truth.Split(' ', [System.StringSplitOptions]::RemoveEmptyEntries)) { $truthArgs += '--truth'; $truthArgs += $t }
& $py (Join-Path $laptop 'analyze_flight.py') --run $run.FullName @truthArgs
if ($LASTEXITCODE -ne 0) { throw "analyze_flight.py failed with exit code $LASTEXITCODE" }

$png = Join-Path $run.FullName 'analysis\flight_overview.png'
if (-not (Test-Path $png)) { throw "Expected picture not found: $png" }
Write-Host ""
Write-Host "Run folder : $($run.FullName)"
Write-Host "Picture    : $png"
Write-Host "Summary    : $(Join-Path $run.FullName 'analysis\flight_summary.md')"
if (-not $NoOpen) { Invoke-Item $png }
