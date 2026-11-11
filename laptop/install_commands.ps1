# Puts the laptop tools on PATH so they run from any folder:
#   run_tag_mission --drone-id 1 --config tag_layout.yaml --simulate
#   live_map --config tag_layout.yaml --drone-id 1
#   tag_stream --esp-ip 192.168.1.222
#   send_trajectory ... / analyze_flight ...
#   tag_layout_editor            (opens the HTML page)
# Each command is a small .cmd in %USERPROFILE%\.local\bin that runs the script in this
# folder with `python`. Re-run this if the repo moves. A bare --config name is also looked
# up in laptop\ and Downloads (mission_config.find_config).
$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$bin = Join-Path $HOME ".local\bin"
New-Item -ItemType Directory -Force $bin | Out-Null

foreach ($name in "run_tag_mission", "live_map", "tag_stream", "send_trajectory", "analyze_flight", "simple_arm_takeoff_land") {
    $script = Join-Path $here "$name.py"
    if (-not (Test-Path $script)) { continue }
    Set-Content -Encoding ascii (Join-Path $bin "$name.cmd") "@echo off`r`npython `"$script`" %*"
    Write-Host "  $name -> $script"
}
$page = Join-Path $here "tag_layout_editor.html"
Set-Content -Encoding ascii (Join-Path $bin "tag_layout_editor.cmd") "@echo off`r`nstart `"`" `"$page`""
Write-Host "  tag_layout_editor -> $page"

$userPath = [Environment]::GetEnvironmentVariable("Path", "User")
if (($userPath -split ";") -notcontains $bin) {
    [Environment]::SetEnvironmentVariable("Path", "$userPath;$bin", "User")
    Write-Host "Added $bin to your PATH: open a new terminal."
}
