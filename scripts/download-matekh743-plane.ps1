#Requires -Version 5.1
<#
.SYNOPSIS
  Download ArduPlane MatekH743 stable firmware (with_bl.hex + .apj).

.DESCRIPTION
  Fetches official prebuilt binaries into firmware/Plane/stable/MatekH743/.
  Use arduplane_with_bl.hex for first DFU flash; arduplane.apj for later updates.

.PARAMETER Channel
  Firmware channel: stable (default) or latest.

.PARAMETER Bdshot
  If set, download MatekH743-bdshot instead of MatekH743.
#>
[CmdletBinding()]
param(
    [ValidateSet("stable", "latest")]
    [string]$Channel = "stable",

    [switch]$Bdshot
)

$ErrorActionPreference = "Stop"

$Board = if ($Bdshot) { "MatekH743-bdshot" } else { "MatekH743" }
$BaseUrl = "https://firmware.ardupilot.org/Plane/$Channel/$Board"
# scripts/ lives directly under the repo root
$RepoRoot = Split-Path -Parent $PSScriptRoot
$OutDir = Join-Path $RepoRoot "firmware\Plane\$Channel\$Board"
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

# name -> minimum expected size in bytes (metadata files are tiny)
$Files = [ordered]@{
    "arduplane_with_bl.hex" = 100000
    "arduplane.apj"         = 100000
    "firmware-version.txt"  = 8
    "git-version.txt"       = 8
}

Write-Host "Source : $BaseUrl"
Write-Host "Output : $OutDir"
Write-Host ""

foreach ($Name in $Files.Keys) {
    $MinSize = $Files[$Name]
    $Url = "$BaseUrl/$Name"
    $Dest = Join-Path $OutDir $Name
    Write-Host "Downloading $Name ..."
    try {
        Invoke-WebRequest -Uri $Url -OutFile $Dest -UseBasicParsing
    }
    catch {
        throw "Failed to download $Url : $($_.Exception.Message)"
    }

    $Size = (Get-Item $Dest).Length
    if ($Size -lt $MinSize) {
        throw "Downloaded file looks too small: $Dest ($Size bytes, expected >= $MinSize)"
    }
    Write-Host "  OK ($Size bytes) -> $Dest"
}

Write-Host ""
Write-Host "Done."
Write-Host "First flash (DFU):  $(Join-Path $OutDir 'arduplane_with_bl.hex')"
Write-Host "Later update (MP):  $(Join-Path $OutDir 'arduplane.apj')"

$VersionFile = Join-Path $OutDir "firmware-version.txt"
if (Test-Path $VersionFile) {
    Write-Host "Version: $((Get-Content $VersionFile -Raw).Trim())"
}
