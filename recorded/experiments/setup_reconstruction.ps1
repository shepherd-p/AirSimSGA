$ErrorActionPreference = 'Stop'
$agBasePython = 'C:/Users/Stephen/.conda/envs/projectairsim310/python.exe'
$agEnv = Join-Path $PSScriptRoot '.venv_reconstruction'
$agReconPython = Join-Path $agEnv 'Scripts/python.exe'
if (-not (Test-Path -LiteralPath $agReconPython)) {
    & $agBasePython -m venv $agEnv
    if ($LASTEXITCODE -ne 0) { throw 'Could not create isolated reconstruction environment' }
}
& $agReconPython -m pip install 'open3d==0.19.0'
if ($LASTEXITCODE -ne 0) { throw 'Isolated Open3D installation failed' }
Write-Host 'Reconstruction environment ready; original ProjectAirSim Python environment unchanged.'
