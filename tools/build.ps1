# Builds the SSV modules (and optionally the Shuttle FDO MFD) from the sources.
#
# Needs: Visual Studio or the Build Tools (C++, x86), the Orbiter SDK (an Orbiter 2024 folder, or the folder made by get-orbiter-sdk.ps1; it is only read)
# and, for SSV_OV, the ATL static libraries (if the Visual Studio installation does not have them, get-atl.ps1 gets them).
#
# Usage: .\build.ps1 -Orbiter "C:\Games\Orbiter 2024" [-Threads 1] [-Priority BelowNormal] [-Toolset v145] [-WinSdk 10.0.22621.0]
#                    [-Fdo <FDO repository folder>] [-Tests]
# The modules are written to <repository>\Modules, the FDO plugin to <FDO repository>\Modules\Plugin.
# assemble.ps1 puts them in an Orbiter folder, package.ps1 makes the zip of a development build.
#
# The projects of the repository use the toolset v141; -Toolset replaces it (v143 for Visual Studio 2022, v145 for Visual Studio 2026).
# On a normal PC use the default of -Threads and -Priority (one thread, low priority): the PC stays usable. The pipeline uses all the cores.
param(
	[Parameter(Mandatory = $true)][string]$Orbiter,
	[int]$Threads = 1,
	[ValidateSet('Idle', 'BelowNormal', 'Normal')][string]$Priority = 'BelowNormal',
	[string]$Toolset = 'v145',
	[string]$WinSdk = '10.0.22621.0',
	[string]$Fdo = '',
	[string]$Atl = '',
	[string]$Configuration = 'Release',
	[switch]$Tests
)

$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
if ($Atl -eq '') { $Atl = Join-Path $here '..\..\atl' }
$repo = (Resolve-Path (Join-Path $here '..')).Path
$ssv = Join-Path $repo 'Orbitersdk\Space Shuttle Vessel'
$sdk = Join-Path $repo 'Orbitersdk'
$orbsdk = Join-Path $Orbiter 'Orbitersdk'
if (-not (Test-Path (Join-Path $orbsdk 'include\OrbiterAPI.h'))) { throw "Orbiter SDK not found in $orbsdk" }

$msbuild = Get-ChildItem -Path "${env:ProgramFiles(x86)}\Microsoft Visual Studio\*\*\MSBuild\Current\Bin\MSBuild.exe", "${env:ProgramFiles}\Microsoft Visual Studio\*\*\MSBuild\Current\Bin\MSBuild.exe" -ErrorAction SilentlyContinue | Select-Object -First 1
if (-not $msbuild) { throw 'MSBuild.exe not found' }

# the project expects the SDK folders next to the SSV sources: links to the Orbiter installation (it is not modified)
foreach ($d in 'include', 'lib', 'XRSound')
{
	$link = Join-Path $sdk $d
	if (-not (Test-Path $link)) { New-Item -ItemType Junction -Path $link -Target (Join-Path $orbsdk $d) | Out-Null }
}

# ATL static libraries: the ones of the Visual Studio installation, or the ones that get-atl.ps1 extracts from the packages of the installer
$build = Join-Path $repo 'build'
New-Item -ItemType Directory -Force $build | Out-Null
$installedAtl = Get-ChildItem -Path "${env:ProgramFiles(x86)}\Microsoft Visual Studio\*\*\VC\Tools\MSVC\14.*\atlmfc\lib\x86\atls.lib", "${env:ProgramFiles}\Microsoft Visual Studio\*\*\VC\Tools\MSVC\14.*\atlmfc\lib\x86\atls.lib" -ErrorAction SilentlyContinue | Select-Object -First 1
$libdirs = ''
if ($installedAtl)
{
	Write-Host "ATL: $($installedAtl.FullName)"
}
else
{
	if (-not (Test-Path (Join-Path $Atl 'lib\x86\atls.lib'))) { & (Join-Path $here 'get-atl.ps1') -Out $Atl }
	$libdirs = (Resolve-Path (Join-Path $Atl 'lib\x86')).Path + ';'
}
$props = Join-Path $build 'extra.props'
@"
<Project xmlns="http://schemas.microsoft.com/developer/msbuild/2003">
  <ItemDefinitionGroup>
    <Link>
      <AdditionalLibraryDirectories>${libdirs}%(AdditionalLibraryDirectories)</AdditionalLibraryDirectories>
    </Link>
  </ItemDefinitionGroup>
</Project>
"@ | Set-Content -Path $props -Encoding UTF8

function Invoke-LowPriorityBuild( [string]$name, [string[]]$arguments )
{
	$out = Join-Path $build "$name.out"
	$p = Start-Process -FilePath $msbuild.FullName -ArgumentList $arguments -PassThru -WindowStyle Hidden -RedirectStandardOutput $out
	$null = $p.Handle # keeps the process handle, otherwise ExitCode can be empty after the wait
	try { $p.PriorityClass = $Priority } catch { }
	$p.WaitForExit()
	if ($p.ExitCode -ne 0)
	{
		if (Test-Path $out) { Get-Content $out | Select-String -Pattern 'error|fatal' | Select-Object -First 40 | ForEach-Object { Write-Host $_.Line } }
		throw "Build of $name failed, see $out"
	}
	Write-Host "${name}: OK"
}

$targets = 'SSV_OV;SSV_ET;SSV_SRB;SSV_LCC;SSV_LC39;SSV_MLP;SSV_CT;SSV_DragChute;SSV_XenonLights;SSV_Centaur;SSV_IUS_SRM1;SSV_IUS_SRM2;SSV_OBSS;SSV_SLC6;SSV_HST'
Invoke-LowPriorityBuild 'ssv' @(
	"`"$ssv\SpaceShuttleVessel_2017.sln`"", "/t:$targets", "/p:Configuration=$Configuration", '/p:Platform=x86',
	"/p:PlatformToolset=$Toolset", "/p:WindowsTargetPlatformVersion=$WinSdk", "/p:ForceImportAfterCppProps=`"$props`"",
	"/m:$Threads", '/nr:false', '/nologo', '/v:minimal', "/flp:logfile=`"$(Join-Path $build 'ssv.log')`";verbosity=normal"
)

if ($Fdo -ne '')
{
	$fdoproj = Join-Path $Fdo 'Orbitersdk\samples\ShuttleFDOMFD\ShuttleFDOMFD.vcxproj'
	$fdosdk = Join-Path $Fdo 'Orbitersdk'
	foreach ($d in 'include', 'lib')
	{
		$link = Join-Path $fdosdk $d
		if (-not (Test-Path $link)) { New-Item -ItemType Junction -Path $link -Target (Join-Path $orbsdk $d) | Out-Null }
	}
	Invoke-LowPriorityBuild 'fdo' @(
		"`"$fdoproj`"", "/p:Configuration=$Configuration", '/p:Platform=Win32', "/p:PlatformToolset=$Toolset",
		"/p:WindowsTargetPlatformVersion=$WinSdk", "/m:$Threads", '/nr:false', '/nologo', '/v:minimal'
	)
}

if ($Tests)
{
	& (Join-Path $ssv 'tests\run_tests.ps1') -SdkInclude (Join-Path $orbsdk 'include')
	if ($LASTEXITCODE -ne 0) { throw 'the tests failed' }
}
Write-Host 'Done.'
