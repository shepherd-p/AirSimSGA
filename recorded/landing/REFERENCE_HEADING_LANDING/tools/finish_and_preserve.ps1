$ErrorActionPreference='Stop'
$revisionRoot=Split-Path $PSScriptRoot -Parent
$nativeRoot=Split-Path $revisionRoot -Parent
$receipt=Get-Content -Raw -LiteralPath (Join-Path $revisionRoot 'runtime_receipt.json') | ConvertFrom-Json
$process=Get-Process -Id $receipt.pid -ErrorAction SilentlyContinue
if ($process) {
  $epoch=[DateTime]'1970-01-01'
  if ([Math]::Abs(($process.StartTime.ToUniversalTime()-$epoch).TotalSeconds-$receipt.create_time) -gt 2) {throw 'Simulator PID identity changed; will not stop'}
  Stop-Process -Id $process.Id
}
Copy-Item -LiteralPath (Join-Path $revisionRoot 'preserved_parent_runtime_receipt.json') -Destination (Join-Path $nativeRoot 'runtime_receipt.json') -Force
Write-Output 'Stopped this revision simulator and restored the previous delivery runtime receipt'
