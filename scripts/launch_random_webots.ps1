<#
.SYNOPSIS
    AetherSwarm Randomized Webots Run Launcher.

.DESCRIPTION
    Generates a new random 10-POI scenario using the authoritative simulation
    pipeline, exports an immutable simulation trace, and launches Webots R2025a
    to visualize the newly generated scenario.

.PARAMETER Seed
    Random integer seed. If omitted or 0, a new random seed is generated automatically.

.PARAMETER NumPois
    Number of POIs to generate (default: 10).

.PARAMETER MaxTicks
    Maximum simulation ticks to run (default: 2700).

.PARAMETER NoLaunch
    Generate scenario and export trace without launching Webots GUI (useful for tests/dry runs).

.EXAMPLE
    .\scripts\launch_random_webots.ps1
    Generates a new random seed, runs authoritative simulation, and opens Webots.

.EXAMPLE
    .\scripts\launch_random_webots.ps1 -Seed 45892
    Runs reproducible random scenario with specified seed and opens Webots.

.EXAMPLE
    .\scripts\launch_random_webots.ps1 -NoLaunch
    Generates scenario and trace only without opening Webots.
#>

[CmdletBinding()]
param(
    [Parameter(Mandatory = $false)]
    [int]$Seed = 0,

    [Parameter(Mandatory = $false)]
    [int]$NumPois = 10,

    [Parameter(Mandatory = $false)]
    [int]$MaxTicks = 2700,

    [Parameter(Mandatory = $false)]
    [switch]$NoLaunch
)

$ErrorActionPreference = "Stop"

Write-Host "================================================================================" -ForegroundColor Cyan
Write-Host " AETHERSWARM RANDOMIZED WEBOTS WORKFLOW LAUNCHER" -ForegroundColor Cyan
Write-Host " Authoritative working-model pipeline with full-arena POI randomization" -ForegroundColor Cyan
Write-Host "================================================================================" -ForegroundColor Cyan

# 1. Resolve Repository Root
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = (Get-Item $ScriptDir).Parent.FullName

# 2. Determine / Generate Random Seed
if ($Seed -eq 0) {
    # Generate positive random integer (avoiding 2026 baseline for distinct new run)
    do {
        $Seed = Get-Random -Minimum 1000 -Maximum 999999
    } while ($Seed -eq 2026)
    Write-Host "[Launcher] Auto-generated new random seed : $Seed" -ForegroundColor Green
} else {
    Write-Host "[Launcher] Using user-specified seed      : $Seed" -ForegroundColor Yellow
}

# 3. Clean stale Webots project cache so camera/window state starts fresh
$WbprojPath = Join-Path $RepoRoot "visualization\webots\worlds\.uavx_round1.wbproj"
if (Test-Path $WbprojPath) {
    Write-Host "[Launcher] Removing stale project cache  : .uavx_round1.wbproj" -ForegroundColor DarkGray
    Remove-Item -Path $WbprojPath -Force -ErrorAction SilentlyContinue
}

# 4. Define Target Output Paths (using .tmp for scenario to keep git tree clean)
$ScenarioTmpRel = "visualization/webots/data/random_scenario.tmp"
$TraceRel       = "visualization/webots/data/random_scenario_trace.json"
$TraceFullPath  = Join-Path $RepoRoot "visualization\webots\data\random_scenario_trace.json"

Write-Host "[Launcher] POI Count                     : $NumPois (Full 1000m x 1000m Arena)" -ForegroundColor White
Write-Host "[Launcher] Simulation Duration           : $MaxTicks ticks" -ForegroundColor White
Write-Host "[Launcher] Running authoritative simulation pipeline..." -ForegroundColor Yellow

# 5. Execute Authoritative Generation & Trace Export Pipeline via WSL
$WslDistro = "Ubuntu-24.04"
$WslWorkDir = "/home/dell/swarm_ws/AetherSwarm"

wsl -d $WslDistro --cd $WslWorkDir .venv/bin/python scripts/generate_scenario.py `
    --seed $Seed `
    --num-pois $NumPois `
    --full-arena `
    --max-ticks $MaxTicks `
    --output-trace $TraceRel `
    --output-scenario $ScenarioTmpRel

if ($LASTEXITCODE -ne 0) {
    Write-Error "[Launcher] Scenario generation and trace export failed with exit code $LASTEXITCODE."
    exit $LASTEXITCODE
}

# 6. Verify Generated Trace Integrity
if (-not (Test-Path $TraceFullPath)) {
    Write-Error "[Launcher] Expected trace file was not created: $TraceFullPath"
    exit 1
}

# Parse and display metadata directly from newly generated trace JSON
$TraceRaw = Get-Content -Path $TraceFullPath -Raw | ConvertFrom-Json
$Meta = $TraceRaw.metadata

Write-Host "`n--- Generated Trace Verification ---" -ForegroundColor Cyan
Write-Host "  Scenario Name       : $($Meta.scenario_name)" -ForegroundColor White
Write-Host "  Metadata Seed       : $($Meta.seed)" -ForegroundColor Green
Write-Host "  Total Ticks         : $($Meta.total_ticks)" -ForegroundColor White
Write-Host "  UAV Fleet Count     : $($Meta.uav_ids.Count) ($($Meta.uav_ids -join ', '))" -ForegroundColor White
Write-Host "  Task Count          : $($Meta.task_ids.Count) ($($Meta.task_ids -join ', '))" -ForegroundColor White
Write-Host "  Min Separation Spec : $($Meta.min_separation_m)m" -ForegroundColor White

if ($Meta.seed -ne $Seed) {
    Write-Error "[Launcher] Trace metadata seed ($($Meta.seed)) does not match requested seed ($Seed)!"
    exit 1
}

if ($Meta.task_ids.Count -ne $NumPois) {
    Write-Error "[Launcher] Generated POI count ($($Meta.task_ids.Count)) does not match requested ($NumPois)!"
    exit 1
}

# 7. Set Scenario Selector Environment Variable
$env:AETHERSWARM_SCENARIO = "random"
Write-Host "[Launcher] Environment set               : AETHERSWARM_SCENARIO=random" -ForegroundColor Green

# 8. Locate Webots Executable
$PreferredWebots = "C:\Program Files\Webots\msys64\mingw64\bin\webotsw.exe"
$FallbackWebots  = "C:\Program Files\Webots\webotsw.exe"

$WebotsExe = $null
if (Test-Path $PreferredWebots) {
    $WebotsExe = $PreferredWebots
} elseif (Test-Path $FallbackWebots) {
    $WebotsExe = $FallbackWebots
} else {
    Write-Error "[Launcher] Webots GUI executable (webotsw.exe) not found at expected locations."
    exit 1
}

# 9. Resolve World File Path (Prefer Z: mapped drive for Webots Windows process)
$ZPath = "Z:\home\dell\swarm_ws\AetherSwarm\visualization\webots\worlds\uavx_round1.wbt"
if (Test-Path $ZPath) {
    $WorldPath = $ZPath
} else {
    $WorldPath = (Resolve-Path (Join-Path $RepoRoot "visualization\webots\worlds\uavx_round1.wbt")).ProviderPath
}

if (-not (Test-Path $WorldPath)) {
    Write-Error "[Launcher] World file not found: $WorldPath"
    exit 1
}

# 10. Launch Webots GUI
if ($NoLaunch) {
    Write-Host "`n[Launcher] -NoLaunch specified. Trace is ready. Webots GUI not launched." -ForegroundColor Yellow
} else {
    Write-Host "`n[Launcher] Launching Webots GUI..." -ForegroundColor Green
    Write-Host "  Binary : $WebotsExe" -ForegroundColor DarkGray
    Write-Host "  World  : $WorldPath" -ForegroundColor DarkGray

    $WebotsDir = Split-Path -Parent $WebotsExe
    Start-Process -FilePath $WebotsExe -ArgumentList "`"$WorldPath`"" -WorkingDirectory $WebotsDir
    Write-Host "[Launcher] Webots process initiated successfully." -ForegroundColor Green
}

Write-Host "================================================================================" -ForegroundColor Cyan
