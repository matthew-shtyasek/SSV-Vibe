# Gets what the build of SSV needs from the Orbiter 2024 release (x86): the SDK headers and libraries and the XRSound SDK.
# Only these folders are extracted, not the whole simulator: <Out>\Orbitersdk\include, <Out>\Orbitersdk\lib, <Out>\Orbitersdk\XRSound.
# The result is used by build.ps1 (-Orbiter <Out>). Used by the build pipeline, it also works on a PC without Orbiter.
#
# Usage: .\get-orbiter-sdk.ps1 [-Out orbiter-sdk] [-Url <Orbiter-x86.zip of the release>]
param(
	[string]$Out = 'orbiter-sdk',
	[string]$Url = 'https://github.com/orbitersim/orbiter/releases/download/2024/Orbiter-x86.zip'
)

$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.IO.Compression.FileSystem

$zipfile = Join-Path ([System.IO.Path]::GetTempPath()) 'Orbiter-x86.zip'
if (-not (Test-Path $zipfile))
{
	Write-Host "Downloading $Url"
	# curl.exe is much faster than Invoke-WebRequest for a file of this size
	& curl.exe -L --fail --retry 3 -o $zipfile $Url
	if ($LASTEXITCODE -ne 0) { throw 'download of the Orbiter release failed' }
}

New-Item -ItemType Directory -Force $Out | Out-Null
$wanted = 'Orbitersdk/include/', 'Orbitersdk/lib/', 'Orbitersdk/XRSound/'
$count = 0
$zip = [System.IO.Compression.ZipFile]::OpenRead($zipfile)
try
{
	foreach ($e in $zip.Entries)
	{
		$name = $e.FullName.Replace('\', '/')
		if ($name.EndsWith('/')) { continue }
		# the release may have a folder above Orbitersdk
		$i = $name.IndexOf('Orbitersdk/')
		if ($i -lt 0) { continue }
		$rel = $name.Substring($i)
		$ok = $false
		foreach ($w in $wanted) { if ($rel.StartsWith($w)) { $ok = $true } }
		if (-not $ok) { continue }
		$dest = Join-Path $Out ($rel.Replace('/', '\'))
		New-Item -ItemType Directory -Force (Split-Path $dest) | Out-Null
		[System.IO.Compression.ZipFileExtensions]::ExtractToFile($e, $dest, $true)
		$count++
	}
}
finally { $zip.Dispose() }

foreach ($f in 'Orbitersdk\include\OrbiterAPI.h', 'Orbitersdk\lib\Orbiter.lib', 'Orbitersdk\XRSound\XRSound.h', 'Orbitersdk\XRSound\XRSound.lib')
{
	if (-not (Test-Path (Join-Path $Out $f))) { throw "$f was not found in the Orbiter release" }
}
Write-Host "$count files of the Orbiter SDK are in $Out"
