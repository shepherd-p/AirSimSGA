param(
    [switch]$Visible,
    [switch]$Offscreen,
    [switch]$ExportZoneGraph,
    [switch]$ControlledTraffic,
    [ValidateSet('Big','Small')][string]$Map = 'Big'
)

$ErrorActionPreference = 'Stop'
if ($Visible -and $Offscreen) { throw 'Choose either -Visible or -Offscreen.' }
$workspace = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$editor = Join-Path $workspace 'UE_5.2/Engine/Binaries/Win64/UnrealEditor.exe'
$project = Join-Path $workspace 'CitySample/CitySample.uproject'
$mapName = if ($Map -eq 'Big') { 'Big_City_LVL' } else { 'Small_City_LVL' }
$level = Join-Path $workspace "CitySample/Content/Map/$mapName.umap"
$plugin = Join-Path $workspace 'CitySample/Plugins/ProjectAirSim/ProjectAirSim.uplugin'
$shaderDir = Join-Path $PSScriptRoot 'runtime_cache/shaders'
$ddcDir = Join-Path $PSScriptRoot 'runtime_cache/ddc'
$log = Join-Path $PSScriptRoot 'citysample_ue_session.log'
foreach ($required in @($editor, $project, $level, $plugin)) {
    if (-not (Test-Path -LiteralPath $required -PathType Leaf)) { throw "Missing: $required" }
}
foreach ($port in @(8989,8990)) {
    $client = [System.Net.Sockets.TcpClient]::new()
    try {
        $pending = $client.ConnectAsync('127.0.0.1', $port)
        try { $null = $pending.Wait(500) } catch { }
        if ($client.Connected) { throw "ProjectAirSim port $port is in use." }
    } finally { $client.Dispose() }
}
New-Item -ItemType Directory -Path $shaderDir,$ddcDir -Force | Out-Null
[Environment]::SetEnvironmentVariable('UE-LocalDataCachePath', $ddcDir, 'Process')
$mapUrl = "/Game/Map/$mapName`?game=/Script/ProjectAirSim.ProjectAirSimGameMode"
$arguments = @('"' + $project + '"', $mapUrl, '-game', '-nosplash', '-nosound',
    '-windowed', '-ResX=1280', '-ResY=720',
    '-ShaderWorkingDir="' + $shaderDir + '"',
    '-LocalDataCachePath="' + $ddcDir + '"',
    '-abslog="' + $log + '"')
if ($Offscreen) { $arguments += '-RenderOffscreen' }
$commands = @()
if ($ControlledTraffic) {
    $commands += 'MassTraffic.NumTrafficVehiclesScale 0'
    $commands += 'ai.mass.scalability.SpawnDensityMultiplier 0'
}
if ($ExportZoneGraph) { $commands += 'AG.ZoneGraph.Export' }
if ($commands.Count -gt 0) { $arguments += '-ExecCmds="' + ($commands -join ',') + '"' }
$windowStyle = if ($Visible) { 'Normal' } else { 'Hidden' }
$process = Start-Process -FilePath $editor -ArgumentList $arguments `
    -WorkingDirectory (Split-Path $project -Parent) -WindowStyle $windowStyle -PassThru
Write-Output "City Sample $Map map started with ProjectAirSimGameMode, PID $($process.Id). Log: $log"
