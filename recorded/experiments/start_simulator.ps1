param([switch]$Offscreen)
$ErrorActionPreference = 'Stop'
$agRoot = $PSScriptRoot
$agWorkspace = Split-Path $agRoot -Parent
$agEditor = Join-Path $agWorkspace 'UE_5.2/Engine/Binaries/Win64/UnrealEditor.exe'
$agProject = Join-Path $agWorkspace 'CityPark1/CityPark1.uproject'
if (-not (Test-Path -LiteralPath $agEditor)) { throw "Missing existing UE editor: $agEditor" }
if (-not (Test-Path -LiteralPath $agProject)) { throw "Missing existing project: $agProject" }
foreach ($agPort in @(8989,8990)) {
    $agProbe = [System.Net.Sockets.TcpClient]::new()
    try {
        $agPending = $agProbe.ConnectAsync('127.0.0.1', $agPort)
        try { $null = $agPending.Wait(500) } catch { }
        if ($agProbe.Connected) { throw "ProjectAirSim port $agPort is already in use. Use the running simulator; do not start a second one." }
    } finally { $agProbe.Dispose() }
}
$agArgs = @('"' + $agProject + '"', '-game', '-nosplash', '-nosound', '-windowed', '-ResX=1280', '-ResY=720',
            '-abslog="' + (Join-Path $agRoot 'verification/ue_session.log') + '"')
if ($Offscreen) { $agArgs += '-RenderOffscreen' }
$agProcess = Start-Process -FilePath $agEditor -ArgumentList $agArgs -WorkingDirectory $agRoot -WindowStyle Hidden -PassThru
$agProcess.Id | Set-Content -LiteralPath (Join-Path $agRoot 'verification/launched_pid.txt')
Write-Host "Started existing CityPark1 binary, PID $($agProcess.Id). Wait for ports 8989/8990, then run preflight.py."
