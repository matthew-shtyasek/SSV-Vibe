# Builds and runs the tests. Needs the Visual Studio build tools (C++, x86) and the Orbiter SDK include folder.
# Usage: .\run_tests.ps1 [-SdkInclude <path to Orbitersdk\include>]
param(
	[string]$SdkInclude = ''
)

$ErrorActionPreference = 'Stop'
if ($SdkInclude -eq '') { $SdkInclude = Join-Path (Split-Path -Parent $MyInvocation.MyCommand.Path) '..\..\include' }
$vcvars = Get-ChildItem -Path "${env:ProgramFiles(x86)}\Microsoft Visual Studio\*\*\VC\Auxiliary\Build\vcvars32.bat", "${env:ProgramFiles}\Microsoft Visual Studio\*\*\VC\Auxiliary\Build\vcvars32.bat" -ErrorAction SilentlyContinue | Select-Object -First 1
if (-not $vcvars) { throw 'vcvars32.bat not found: install the Visual Studio build tools (C++)' }
if (-not (Test-Path (Join-Path $SdkInclude 'OrbiterAPI.h'))) { throw "OrbiterAPI.h not found in $SdkInclude (use -SdkInclude)" }

$out = Join-Path $env:TEMP 'ssv_tests'
New-Item -ItemType Directory -Force $out | Out-Null
$src = Join-Path $PSScriptRoot 'rms_math_tests.cpp'
$exe = Join-Path $out 'rms_math_tests.exe'
$ssvlib = Join-Path $PSScriptRoot '..\SSVlib\include'

# MathSSV.cpp has functions that call Orbiter (not needed by the tests): their symbols stay unresolved
$bat = Join-Path $out 'build_tests.bat'
@(
	'@echo off',
	"call `"$($vcvars.FullName)`" >nul 2>&1",
	"cl /nologo /EHsc /std:c++17 /DWIN32 /Fo`"$out\rms_math_tests.obj`" /Gy /Fe`"$exe`" /I`"$SdkInclude`" /I`"$ssvlib`" `"$src`" /link /FORCE:UNRESOLVED /IGNORE:4088"
) | Set-Content -Path $bat -Encoding ASCII
& $bat
if ($LASTEXITCODE -ne 0) { throw 'build of the tests failed' }
& $exe
exit $LASTEXITCODE
