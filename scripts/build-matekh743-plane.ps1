#Requires -Version 5.1
<#
.SYNOPSIS
  Optional local WSL build of patched ArduPlane MatekH743.

.DESCRIPTION
  Prefer GitHub Actions: push, then .\scripts\download-thstfac-plane.ps1
  Native Windows cannot run ArduPilot waf. This wrapper calls the bash script
  inside WSL. Install WSL2 Ubuntu first: wsl --install -d Ubuntu
#>
[CmdletBinding()]
param()

$ErrorActionPreference = "Stop"
$RepoRoot = Split-Path -Parent $PSScriptRoot
# D:\code\foo -> /mnt/d/code/foo (Windows PowerShell 5.1 has no scriptblock -replace)
$drive = $RepoRoot.Substring(0, 1).ToLowerInvariant()
$unixPath = ($RepoRoot.Substring(2) -replace '\\', '/')
$BashScript = "/mnt/$drive$unixPath/scripts/build-matekh743-plane.sh"

if (-not (Get-Command wsl -ErrorAction SilentlyContinue)) {
    throw "WSL not found. Install Ubuntu: wsl --install -d Ubuntu"
}

Write-Host "WSL $BashScript"
wsl -e bash -lc "chmod +x '$BashScript' && '$BashScript'"
if ($LASTEXITCODE -ne 0) {
    throw "build failed (WSL exit $LASTEXITCODE)"
}
