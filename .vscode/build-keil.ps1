param([switch]$Rebuild)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$keilExe = 'E:\stm32\keli5\UV4\UV4.exe'
$projectFile = Join-Path $projectRoot '1.uvprojx'
$logFile = Join-Path $projectRoot 'Objects\vscode-build.log'
$buildFlag = if ($Rebuild) { '-r' } else { '-b' }

if (!(Test-Path -LiteralPath $keilExe)) {
    throw "Keil executable not found: $keilExe"
}
New-Item -ItemType Directory -Path (Split-Path -Parent $logFile) -Force | Out-Null
if (Test-Path -LiteralPath $logFile) {
    Remove-Item -LiteralPath $logFile
}
Write-Host "Building Target 1..."
$process = Start-Process -FilePath $keilExe -ArgumentList @($buildFlag, ('"' + $projectFile + '"'), '-t', '"Target 1"', '-o', ('"' + $logFile + '"')) -WorkingDirectory $projectRoot -WindowStyle Hidden -PassThru
if (!$process.WaitForExit(60000)) {
    Write-Error "Keil has not finished after 60 seconds. Check Keil for a dialog. Log: $logFile"
    exit 2
}
if (!(Test-Path -LiteralPath $logFile)) {
    throw "Keil did not create a build log. Exit code: $($process.ExitCode)"
}
$buildOutput = Get-Content -LiteralPath $logFile -Raw -Encoding Default
Write-Host $buildOutput
if ($process.ExitCode -gt 1 -or $buildOutput -notmatch '0 Error\(s\)') {
    exit 2
}
exit 0
