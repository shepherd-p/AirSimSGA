$ErrorActionPreference='Stop'
$runRoot=Split-Path $PSScriptRoot -Parent
$labRoot='F:\EpicGamesEnv\LandingWaterLab'
$editor='F:\EpicGamesEnv\UE_5.2\Engine\Binaries\Win64\UnrealEditor-Win64-DebugGame.exe'
$project=Join-Path $labRoot 'CityPark2/CityPark2.uproject'
foreach($port in @(9189,9190)) {
  $socket=[System.Net.Sockets.TcpClient]::new()
  try {try {$null=$socket.ConnectAsync('127.0.0.1',$port).Wait(500)} catch {}; if($socket.Connected){throw "Landing port $port is occupied"}}
  finally {$socket.Dispose()}
}
$shader=Join-Path $labRoot 'ShaderWorkingDir'
$argsList=@(('"'+$project+'"'),'/Game/SGAWaterLab/WaterDiagnostic?game=/Script/ProjectAirSim.ProjectAirSimGameMode','-game','-debug','-nosplash','-nosound','-unattended','-windowed','-ResX=960','-ResY=540','-RenderOffscreen','-topicsport=9189','-servicesport=9190','-ddc=MovingDeckCache',('-ShaderWorkingDir="'+$shader+'"'),('-abslog="'+(Join-Path $runRoot 'simulator.log')+'"'),'-ExecCmds="t.MaxFPS 60,r.VSync 0"')
$process=Start-Process -FilePath $editor -ArgumentList $argsList -WorkingDirectory $labRoot -WindowStyle Hidden -PassThru
@{pid=$process.Id;project=$project;arguments=$argsList;started_utc=[DateTime]::UtcNow.ToString('o')} | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $runRoot 'launch_receipt.json') -Encoding utf8
Write-Output "Started independent landing simulator PID $($process.Id)"
