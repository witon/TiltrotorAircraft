#Requires -Version 5.1
<#
.SYNOPSIS
  Download ArduPlane V4.6.3-thstfac (MatekH743) built on GitHub Actions.

.DESCRIPTION
  Fetches the rolling Release tag firmware-thstfac into
  firmware/Plane/custom/MatekH743/. No local WSL/compiler needed.

.PARAMETER Repo
  GitHub owner/name. Default: origin remote, else witon/TiltrotorAircraft.
#>
[CmdletBinding()]
param(
    [string]$Repo = ""
)

$ErrorActionPreference = "Stop"
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
$ProgressPreference = "SilentlyContinue"
$RepoRoot = Split-Path -Parent $PSScriptRoot
$OutDir = Join-Path $RepoRoot "firmware\Plane\custom\MatekH743"
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

function Get-GitHubRepo {
    param([string]$Fallback)
    if ($Repo) { return $Repo.Trim() }
    Push-Location $RepoRoot
    try {
        $url = git remote get-url origin 2>$null
    }
    catch {
        $url = $null
    }
    Pop-Location
    if ($url) {
        $url = $url.Trim() -replace '\.git$', ''
        if ($url -match 'github\.com[:/]([^/]+)/([^/]+)$') {
            return "$($Matches[1])/$($Matches[2])"
        }
    }
    return $Fallback
}

$GitHubRepo = Get-GitHubRepo -Fallback "witon/TiltrotorAircraft"
$Tag = "firmware-thstfac"
$BaseUrl = "https://github.com/$GitHubRepo/releases/download/$Tag"

$Files = [ordered]@{
    "arduplane.apj"         = 100000
    "arduplane_with_bl.hex" = 100000
    "firmware-version.txt"  = 8
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
        Invoke-WebRequest -Uri $Url -OutFile $Dest -UseBasicParsing -UserAgent "TiltrotorAircraft"
    }
    catch {
        throw @"
Failed to download $Url
$($_.Exception.Message)

The rolling Release may not exist yet. On GitHub: Actions → Build MatekH743 Plane → Run workflow.
After it finishes, tag firmware-thstfac will have the files.
"@
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
Write-Host "GCS version must be ArduPlane V4.6.3-thstfac"

$VersionFile = Join-Path $OutDir "firmware-version.txt"
if (Test-Path $VersionFile) {
    Write-Host "Version: $((Get-Content $VersionFile -Raw).Trim())"
}
