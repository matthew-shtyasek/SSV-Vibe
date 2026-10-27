# Makes the zip of a development build of SSV: the modules built with build.ps1 and the data of the repository, in the layout of the SSV
# release (extract it in the folder of Orbiter 2024, overwriting existing files, as the installation of SSV says).
#
# The Mission Editor is not built here (it is a separate .NET project): to use it, keep the one of the official release of SSV.
#
# With -HstEx <folder of the unpacked add-on "HST(EX) for Orbiter2016"> the files of the add-on that the Hubble of SSV (SSV_HST) needs and that
# are not in the repository (the configurations and modules of its parts, the rest of the meshes and textures) are added, and the package is
# checked: every mesh and module that the configurations of HST_EX name must be in it. -Docs adds files to Doc\SSV-Vibe.
#
# Usage: .\package.ps1 -Version v1.17-dev.1 [-Out dist] [-HstEx <folder>] [-Docs <file>,<file>]
param(
	[Parameter(Mandatory = $true)][string]$Version,
	[string]$Out = 'dist',
	[string]$HstEx = '',
	[string[]]$Docs = @()
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

if ($HstEx -ne '')
{
	# the files of the add-on that are not in the package yet (the ones of SSV, like hst_ssv_fittings.msh, are not overwritten)
	foreach ($d in 'Config\Vessels\HST_EX', 'Modules\HST_EX', 'Meshes\HST_EX', 'Textures\HST_EX', 'Sound', 'Doc\HST_EX')
	{
		$src = Join-Path $HstEx $d
		if (Test-Path $src)
		{
			robocopy $src (Join-Path $stage $d) /E /XC /XN /XO /R:1 /W:1 /NFL /NDL /NJH /NJS /NP | Out-Null
			if ($LASTEXITCODE -ge 8) { throw "robocopy of $d of the HST(EX) failed" }
		}
	}

	# check: the vessels of the parts name a module and a mesh
	$missing = @()
	foreach ($cfg in Get-ChildItem (Join-Path $stage 'Config\Vessels\HST_EX') -Filter '*.cfg')
	{
		foreach ($line in Get-Content $cfg.FullName)
		{
			if ($line -match '^\s*Module\s*=\s*([^;\s]+)') { if (-not (Test-Path (Join-Path $stage ('Modules\' + ($Matches[1] -replace '/', '\') + '.dll')))) { $missing += "$($cfg.Name): module $($Matches[1])" } }
			if ($line -match '^\s*MeshName\s*=\s*([^;\s]+)') { if (-not (Test-Path (Join-Path $stage ('Meshes\' + ($Matches[1] -replace '/', '\') + '.msh')))) { $missing += "$($cfg.Name): mesh $($Matches[1])" } }
		}
	}
	# the meshes that SSV_HST loads
	$srcHst = Join-Path $repo 'Orbitersdk\Space Shuttle Vessel\HST\SSV_HST.cpp'
	foreach ($m in (Select-String -Path $srcHst -Pattern 'AddMesh\("([^"]+)"\)' -AllMatches | ForEach-Object { $_.Matches } | ForEach-Object { $_.Groups[1].Value } | Sort-Object -Unique))
	{
		if (-not (Test-Path (Join-Path $stage ('Meshes\' + ($m -replace '/', '\') + '.msh')))) { $missing += "SSV_HST.cpp: mesh $m" }
	}
	if ($missing.Count -gt 0) { throw ("The package lacks files of the HST(EX):`n" + ($missing -join "`n")) }
	Write-Host 'HST(EX): modules, meshes and configurations of the parts checked'
}
if ($Docs.Count -gt 0)
{
	$docdir = Join-Path $stage 'Doc\SSV-Vibe'
	New-Item -ItemType Directory -Force $docdir | Out-Null
	foreach ($f in $Docs) { Copy-Item $f $docdir -Force }
}

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
