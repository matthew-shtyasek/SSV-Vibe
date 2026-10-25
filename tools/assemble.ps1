# Puts SSV in an Orbiter 2024 folder: the data of the repository (config, meshes, scenarios, ...) and the modules built with build.ps1.
# The Orbiter folder has to be an unpacked release of Orbiter 2024 (x86), it is not created here. Existing files are overwritten (as the
# normal installation of SSV does, including Config\Base.cfg and Config\Earth.cfg).
#
# Usage: .\assemble.ps1 -Orbiter "H:\Orbiter2024" [-Fdo <FDO repository folder>] [-PlanetTextures "C:\Games\OrbiterBase\Textures"]
param(
	[Parameter(Mandatory = $true)][string]$Orbiter,
	[string]$Fdo = '',
	[string]$PlanetTextures = ''
)

$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
if (-not (Test-Path (Join-Path $Orbiter 'Orbiter_ng.exe'))) { throw "$Orbiter is not an Orbiter 2024 folder (Orbiter_ng.exe not found)" }
if (-not (Test-Path (Join-Path $repo 'Modules\SSV_OV.dll'))) { throw 'SSV_OV.dll not found: run build.ps1 first' }

foreach ($d in 'Config', 'Meshes', 'Missions', 'Scenarios', 'Sound', 'Textures', 'Utils', 'Install', 'Doc')
{
	$src = Join-Path $repo $d
	if (Test-Path $src)
	{
		robocopy $src (Join-Path $Orbiter $d) /E /MT:1 /R:1 /W:1 /NFL /NDL /NJH /NJS /NP | Out-Null
		if ($LASTEXITCODE -ge 8) { throw "robocopy of $d failed" }
	}
}
Copy-Item (Join-Path $repo 'SSV Release Notes.txt'), (Join-Path $repo 'SSV-LICENSE.txt'), (Join-Path $repo 'README.md') $Orbiter -Force
Copy-Item (Join-Path $repo 'Modules\SSV_*.dll') (Join-Path $Orbiter 'Modules') -Force

if ($Fdo -ne '')
{
	New-Item -ItemType Directory -Force (Join-Path $Orbiter 'Modules\Plugin') | Out-Null
	Copy-Item (Join-Path $Fdo 'Modules\Plugin\ShuttleFDOMFD.dll') (Join-Path $Orbiter 'Modules\Plugin') -Force
	robocopy (Join-Path $Fdo 'Config') (Join-Path $Orbiter 'Config') /E /MT:1 /R:1 /W:1 /NFL /NDL /NJH /NJS /NP | Out-Null
	if ($LASTEXITCODE -ge 8) { throw 'robocopy of the FDO data failed' }
}

if ($PlanetTextures -ne '')
{
	# planetary texture archives from another folder (key PlanetTexDir of the Orbiter configuration)
	$cfg = Join-Path $Orbiter 'Orbiter_NG.cfg'
	$line = 'PlanetTexDir = ' + $PlanetTextures.TrimEnd('\') + '\'
	if (Test-Path $cfg)
	{
		$text = Get-Content $cfg
		if ($text -match '^\s*PlanetTexDir\s*=') { $text = $text -replace '^\s*PlanetTexDir\s*=.*$', $line } else { $text = @($text[0], $line) + $text[1..($text.Length - 1)] }
		$text | Set-Content $cfg
	}
	else { $line | Set-Content $cfg }
}
Write-Host "SSV is in $Orbiter"
