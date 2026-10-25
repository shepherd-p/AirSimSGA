param([switch]$Offscreen, [switch]$Visible)
$ErrorActionPreference = 'Stop'
if ($Offscreen -and $Visible) { throw 'Choose either -Offscreen or -Visible.' }
$workspace = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$editor = Join-Path $workspace 'UE_5.2/Engine/Binaries/Win64/UnrealEditor.exe'
$project = Join-Path $workspace 'CityPark1/CityPark1.uproject'
$level = Join-Path $workspace 'CityPark1/Content/AGUrban/UrbanBase.umap'
$shaderDir = Join-Path $PSScriptRoot 'runtime_cache/shaders'
$ddcDir = Join-Path $PSScriptRoot 'runtime_cache/ddc'
if (-not (Test-Path -LiteralPath $editor)) { throw "Missing UE 5.2 editor: $editor" }
if (-not (Test-Path -LiteralPath $project)) { throw "Missing CityPark1 project: $project" }
if (-not (Test-Path -LiteralPath $level)) { throw "Missing UrbanBase level: $level" }
New-Item -ItemType Directory -Force -Path $shaderDir,$ddcDir | Out-Null
[Environment]::SetEnvironmentVariable('UE-LocalDataCachePath', $ddcDir, 'Process')
foreach ($port in @(8989,8990)) {
    $client = [System.Net.Sockets.TcpClient]::new()
    try {
        $pending = $client.ConnectAsync('127.0.0.1', $port)
        try { $null = $pending.Wait(500) } catch { }
        if ($client.Connected) { throw "ProjectAirSim port $port is in use; stop that session before starting UrbanBase." }
    } finally { $client.Dispose() }
}
$arguments = @('"' + $project + '"', '/Game/AGUrban/UrbanBase', '-game', '-nosplash', '-nosound',
    '-windowed', '-ResX=1280', '-ResY=720', '-ShaderWorkingDir="' + $shaderDir + '"',
    '-LocalDataCachePath="' + $ddcDir + '"',
    '-abslog="' + (Join-Path $PSScriptRoot 'urban_ue_session.log') + '"')
if ($Offscreen) { $arguments += '-RenderOffscreen' }
if ($Visible) {
    $process = Start-Process -FilePath $editor -ArgumentList $arguments -WorkingDirectory $PSScriptRoot -PassThru
} else {
    $process = Start-Process -FilePath $editor -ArgumentList $arguments -WorkingDirectory $PSScriptRoot -WindowStyle Hidden -PassThru
}
Write-Host "UrbanBase started with existing UE 5.2 binary, PID $($process.Id)."
