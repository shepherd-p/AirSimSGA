param(
    [ValidateSet('vision','static','moving','suite','analyze','audit')][string]$Mode = 'moving',
    [double]$Speed = 0.5,
    [int]$Repeats = 1,
    [string]$InputDir = '',
    [string]$Config = (Join-Path $PSScriptRoot 'configs/landing/study.json')
)
$ErrorActionPreference = 'Stop'
$landingPython = 'C:/Users/Stephen/.conda/envs/projectairsim310/python.exe'
if (-not (Test-Path -LiteralPath $landingPython)) { throw "Python environment is missing: $landingPython" }
Push-Location $PSScriptRoot
try {
    if ($Mode -in @('analyze','audit')) {
        if (-not $InputDir) { throw 'Provide -InputDir with the actual printed experiment directory.' }
        if ($Mode -eq 'analyze') { & $landingPython export_landing_study.py --input-dir $InputDir }
        else { & $landingPython audit_landing_study.py --input-dir $InputDir }
    } else {
        if (-not (Test-Path -LiteralPath $Config)) { throw "Experiment configuration is missing: $Config" }
        $landingSettings = Get-Content -LiteralPath $Config -Raw | ConvertFrom-Json
        foreach ($landingPort in @($landingSettings.port_topics,$landingSettings.port_services)) {
            $landingProbe = [System.Net.Sockets.TcpClient]::new()
            try { $landingProbe.Connect('127.0.0.1',$landingPort) }
            catch { throw "ProjectAirSim is not listening on $landingPort. Start the simulator matching $Config first." }
            finally { $landingProbe.Dispose() }
        }
        switch ($Mode) {
            'vision' { & $landingPython run_landing_study.py --config $Config --speeds 0 --repeats 1 --probe --timeout 30 }
            'static' { & $landingPython run_landing_study.py --config $Config --speeds 0 --repeats $Repeats }
            'moving' { & $landingPython run_landing_study.py --config $Config --speeds $Speed --repeats $Repeats }
            'suite' { & $landingPython run_landing_study.py --config $Config --repeats $Repeats }
        }
    }
    if ($LASTEXITCODE -ne 0) { throw "Program exited with code $LASTEXITCODE. Read the saved manifest: a failed landing is retained as a failed trial." }
} finally { Pop-Location }
