param([ValidateSet('mapping','citysample')][string]$Scene, [switch]$DisableParkedTraffic)
$ErrorActionPreference = 'Stop'
$videoWorkspace = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$videoRoot = Join-Path $videoWorkspace 'ProjectAirSim_AG_Experiments/EXPERIMENT_VIDEOS_20261002'
$videoEditor = Join-Path $videoWorkspace 'UE_5.2/Engine/Binaries/Win64/UnrealEditor.exe'
foreach ($videoPort in @(8989,8990)) {
    $videoSocket = [System.Net.Sockets.TcpClient]::new()
    try {
        try { $null=$videoSocket.ConnectAsync('127.0.0.1',$videoPort).Wait(500) } catch { }
        if ($videoSocket.Connected) { throw "Port $videoPort occupied; reuse compatible instance instead." }
    } finally { $videoSocket.Dispose() }
}
$videoCache = if ($Scene -eq 'mapping') {
    Join-Path $videoWorkspace 'MovingDeckLab/DerivedDataCache'
} else {
    Join-Path $videoWorkspace 'ProjectAirSim_AG_Experiments/citysample_scene/runtime_cache/ddc'
}
[Environment]::SetEnvironmentVariable('UE-LocalDataCachePath',$videoCache,'Process')
$videoProject = Join-Path $videoWorkspace $(if ($Scene -eq 'mapping') {'CityPark1/CityPark1.uproject'} else {'CitySample/CitySample.uproject'})
$videoLevel = if ($Scene -eq 'mapping') {'/Game/CityPark/Maps/Showcase'} else {'/Game/Map/Big_City_LVL'}
$videoLog = Join-Path $videoRoot ($Scene+'_simulator.log')
$videoShaders = Join-Path $videoRoot ('_runtime/'+$Scene+'/shaders')
New-Item -ItemType Directory -Path $videoShaders -Force | Out-Null
$videoArgs = @(('"'+$videoProject+'"'),($videoLevel+'?game=/Script/ProjectAirSim.ProjectAirSimGameMode'),
    '-game','-nosplash','-nosound','-windowed','-ResX=1280','-ResY=720','-RenderOffscreen',
    ('-ShaderWorkingDir="'+$videoShaders+'"'),('-LocalDataCachePath="'+$videoCache+'"'),('-abslog="'+$videoLog+'"'))
if ($Scene -eq 'citysample') {
    $videoCommands = @('MassTraffic.NumTrafficVehiclesScale 0','ai.mass.scalability.SpawnDensityMultiplier 0')
    if ($DisableParkedTraffic) {
        $videoArgs += '-ini:Engine:[SystemSettings]:MassTraffic.NumTrafficVehiclesScale=0,[SystemSettings]:MassTraffic.NumParkedVehiclesScale=0,[SystemSettings]:ai.mass.scalability.SpawnDensityMultiplier=0'
        $videoCommands += 'MassTraffic.NumParkedVehiclesScale 0'
        $videoCommands += 'ai.mass.ForceDespawn'
    }
    $videoArgs += '-ExecCmds="'+($videoCommands -join ',')+'"'
}
$videoProcess = Start-Process -FilePath $videoEditor -ArgumentList $videoArgs -WorkingDirectory (Split-Path $videoProject -Parent) -WindowStyle Hidden -PassThru
@{pid=$videoProcess.Id;project=$videoProject;args=$videoArgs;cache=$videoCache;started_utc=[DateTime]::UtcNow.ToString('o')} |
    ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $videoRoot ($Scene+'_launch_receipt.json')) -Encoding utf8
Write-Output "Started $Scene recorder simulator PID $($videoProcess.Id)"
