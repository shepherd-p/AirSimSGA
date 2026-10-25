param(
    [ValidateSet('preflight','mapping','navigation','tracking','formation','restore','verify')]
    [string]$Task = 'preflight',
    [double]$Duration = 90,
    [switch]$Offline,
    [switch]$Land
)
$ErrorActionPreference = 'Stop'
$agPython = 'C:/Users/Stephen/.conda/envs/projectairsim310/python.exe'
if (-not (Test-Path -LiteralPath $agPython)) { throw 'Existing projectairsim310 environment not found; set agPython in this independent launcher.' }
Push-Location $PSScriptRoot
try {
    if ($Task -eq 'preflight') {
        if ($Offline) { & $agPython preflight.py --offline } else { & $agPython preflight.py }
    } elseif ($Task -eq 'restore') {
        & $agPython restore_scene.py
    } elseif ($Task -eq 'verify') {
        & $agPython verify_preservation.py
    } else {
        $agScript = "run_$Task.py"
        if ($Land) { & $agPython $agScript --duration $Duration --land }
        else { & $agPython $agScript --duration $Duration }
    }
    $agExit = $LASTEXITCODE
} finally { Pop-Location }
exit $agExit
