# Makes the zip of a development build of SSV: the modules built with build.ps1 and the data of the repository, in the layout of the SSV
# release (extract it in the folder of Orbiter 2024, overwriting existing files, as the installation of SSV says).
#
# The Mission Editor is not built here (it is a separate .NET project): to use it, keep the one of the official release of SSV.
#
# Usage: .\package.ps1 -Version v1.17-dev.1 [-Out dist]
param(
	[Parameter(Mandatory = $true)][string]$Version,
	[string]$Out = 'dist'
)

$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$repo = (Resolve-Path (Join-Path $here '..')).Path
if (-not (Test-Path (Join-Path $repo 'Modules\SSV_OV.dll'))) { throw 'SSV_OV.dll not found: run build.ps1 first' }

$name = "SSV-$Version"
$outdir = Join-Path $repo $Out
$stage = Join-Path $outdir $name
if (Test-Path $stage) { Remove-Item -Recurse -Force $stage }
New-Item -ItemType Directory -Force $stage | Out-Null

foreach ($d in 'Config', 'Meshes', 'Missions', 'Scenarios', 'Sound', 'Textures', 'Install', 'Doc')
{
	$src = Join-Path $repo $d
	if (Test-Path $src)
	{
		robocopy $src (Join-Path $stage $d) /E /R:1 /W:1 /NFL /NDL /NJH /NJS /NP | Out-Null
		if ($LASTEXITCODE -ge 8) { throw "robocopy of $d failed" }
	}
}
$modules = Join-Path $stage 'Modules'
New-Item -ItemType Directory -Force $modules | Out-Null
Copy-Item (Join-Path $repo 'Modules\SSV_*.dll') $modules -Force
foreach ($f in 'README.md', 'DEV_BUILD.md', 'DEV_BUILD_RU.md', 'SSV Release Notes.txt', 'SSV-LICENSE.txt')
{
	$p = Join-Path $repo $f
	if (Test-Path $p) { Copy-Item $p $stage -Force }
}

# the controller files of the user are not part of the repository, the examples are
Get-ChildItem (Join-Path $stage 'Config\SSV\Controllers') -Filter 'Joystick*.cfg' -ErrorAction SilentlyContinue | Remove-Item -Force
Remove-Item (Join-Path $stage 'Config\SSV\Controllers\DetectedDevices.txt') -Force -ErrorAction SilentlyContinue

$commit = ''
try { $commit = (& git -C $repo rev-parse --short HEAD) } catch { }
@(
	"Space Shuttle Vessel, development build $Version",
	"Commit: $commit",
	"Built: $((Get-Date).ToUniversalTime().ToString('yyyy-MM-dd HH:mm')) UTC",
	'',
	'Extract this folder in the folder of Orbiter 2024 (x86), overwriting existing files. See DEV_BUILD.md (English) or DEV_BUILD_RU.md (Russian).'
) | Set-Content (Join-Path $stage 'VERSION.txt')

$zip = Join-Path $outdir "$name.zip"
if (Test-Path $zip) { Remove-Item -Force $zip }
Compress-Archive -Path (Join-Path $stage '*') -DestinationPath $zip -CompressionLevel Optimal
Write-Host "$zip ($([math]::Round((Get-Item $zip).Length / 1MB, 1)) MB)"
