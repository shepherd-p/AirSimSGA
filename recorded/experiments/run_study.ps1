param([Parameter(Mandatory=$true)][ValidateSet('navigation','tracking','formation')][string]$Task,
      [switch]$ValidateOnly,[switch]$CaptureOnly,[string]$Config = '')
$ErrorActionPreference = 'Stop'
$studyPython = 'C:/Users/Stephen/.conda/envs/projectairsim310/python.exe'
if (-not (Test-Path -LiteralPath $studyPython)) { throw "Existing Python is missing: $studyPython" }
$studyArguments = @((Join-Path $PSScriptRoot 'run_cooperative_study.py'), $Task)
if ($ValidateOnly) { $studyArguments += '--validate-only' }
if ($CaptureOnly) { $studyArguments += '--capture-only' }
if ($Config) { $studyArguments += @('--config',$Config) }
Push-Location -LiteralPath $PSScriptRoot
try {
    & $studyPython @studyArguments
    if ($LASTEXITCODE -ne 0) { throw "Study stopped with exit code $LASTEXITCODE. Its logs and partial data are preserved." }
} finally { Pop-Location }
