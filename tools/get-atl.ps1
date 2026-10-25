# Gets the ATL static libraries of the installed MSVC toolset, without installing anything in Visual Studio.
# XRSound.lib (used by SSV) was built with static ATL, so linking SSV_OV needs atls.lib, which the Build Tools only have if the
# "C++ ATL" component is installed. The libraries come from the same Microsoft packages that the Visual Studio Installer would use
# (the download is verified with the SHA-256 of the package manifest).
#
# Usage: .\get-atl.ps1 [-Out <folder>]
# The result is <Out>\lib\x86\atls.lib; build.ps1 uses it.
param(
	[string]$Out = ''
)

$ErrorActionPreference = 'Stop'
if ($Out -eq '') { $Out = Join-Path (Split-Path -Parent $MyInvocation.MyCommand.Path) '..\..\atl' }
Add-Type -AssemblyName System.IO.Compression.FileSystem

# installed toolset version
$vsroot = "${env:ProgramFiles(x86)}\Microsoft Visual Studio"
$msvc = Get-ChildItem -Path "$vsroot\*\*\VC\Tools\MSVC\14.*" -Directory -ErrorAction SilentlyContinue | Sort-Object Name -Descending | Select-Object -First 1
if (-not $msvc) { throw 'MSVC toolset not found' }
$version = $msvc.Name
Write-Host "MSVC toolset $version"

if (Test-Path (Join-Path $msvc.FullName 'atlmfc\lib\x86\atls.lib'))
{
	Write-Host 'ATL is already installed in the toolset, nothing to do'
	exit 0
}

# the channel manifest cached by the Visual Studio Installer points to the package manifest
$channel = Get-ChildItem "$env:ProgramData\Microsoft\VisualStudio\Packages\_Channels\*\channelManifest.json" -ErrorAction SilentlyContinue |
	Sort-Object LastWriteTime -Descending | Select-Object -First 1
if (-not $channel) { throw 'channelManifest.json not found (it is created by the Visual Studio Installer)' }
$cm = Get-Content $channel.FullName -Raw | ConvertFrom-Json
$manifestUrl = ($cm.channelItems | Where-Object { $_.type -eq 'Manifest' } | Select-Object -First 1).payloads[0].url

$tmp = Join-Path $env:TEMP 'ssv_atl'
New-Item -ItemType Directory -Force $tmp | Out-Null
$vsman = Join-Path $tmp 'VisualStudio.vsman'
if (-not (Test-Path $vsman)) { Invoke-WebRequest -Uri $manifestUrl -OutFile $vsman }
$man = Get-Content $vsman -Raw | ConvertFrom-Json

$wanted = $man.packages | Where-Object { ($_.id -match '^Microsoft\.VC\.14\.\d+\.\d+\.\d+\.ATL\.(Headers|X86)\.base$') -and ($_.version -eq $version) }
if (-not $wanted) { throw "ATL packages for $version not found in the manifest" }

New-Item -ItemType Directory -Force $Out | Out-Null
foreach ($pkg in $wanted)
{
	foreach ($pl in $pkg.payloads)
	{
		$file = Join-Path $tmp $pl.fileName
		Write-Host "Downloading $($pl.fileName)"
		Invoke-WebRequest -Uri $pl.url -OutFile $file
		$hash = (Get-FileHash $file -Algorithm SHA256).Hash
		if ($hash -ne $pl.sha256.ToUpper()) { throw "SHA-256 of $($pl.fileName) does not match the manifest" }

		$zip = [System.IO.Compression.ZipFile]::OpenRead($file)
		try
		{
			foreach ($e in $zip.Entries)
			{
				$i = $e.FullName.IndexOf('atlmfc/')
				if (($i -lt 0) -or $e.FullName.EndsWith('/')) { continue }
				$dest = Join-Path $Out ($e.FullName.Substring($i + 7).Replace('/', '\'))
				New-Item -ItemType Directory -Force (Split-Path $dest) | Out-Null
				[System.IO.Compression.ZipFileExtensions]::ExtractToFile($e, $dest, $true)
			}
		}
		finally { $zip.Dispose() }
	}
}

if (-not (Test-Path (Join-Path $Out 'lib\x86\atls.lib'))) { throw 'atls.lib was not found in the packages' }
Write-Host "ATL libraries are in $Out"
