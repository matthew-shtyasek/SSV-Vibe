> **Unofficial fork of [GLS-SSV/SSV](https://github.com/GLS-SSV/SSV), development version 1.17-dev (based on SSV v1.16).** This is not an official release of Space Shuttle Vessel and it is not made by the GLS-SSV team.
>
> - **What is added:** the Payload Deployment and Retrieval System (the RMS arm: modes, panel A8U, displays, camera overlay), game controllers (any number of sticks, pedals, gamepads, FPV radios), UNIV PTG tracking, playback of panel events, build scripts, unit tests and a build pipeline. Details, the test list and the builds: [DEV_BUILD.md](DEV_BUILD.md), the builds are in the *Releases* of this repository.
> - **Part of the code was written with the help of AI** (Claude Code). It was built and unit-tested, but not tested for long in the simulator yet. Please report what you find.
> - **License:** GNU GPL v2, as the original (see [SSV-LICENSE.txt](SSV-LICENSE.txt)). The credits of the original are kept below.
>
> **Неофициальный форк [GLS-SSV/SSV](https://github.com/GLS-SSV/SSV), тестовая версия 1.17-dev (на основе SSV v1.16).** Это не официальный релиз Space Shuttle Vessel и не продукт команды GLS-SSV. Что добавлено: рука RMS (режимы, панель A8U, дисплеи, накладка на мониторе), игровые контроллеры, наведение UNIV PTG, запись нажатий в playback, скрипты сборки, тесты и конвейер. Подробности, список проверок и сборки: [DEV_BUILD_RU.md](DEV_BUILD_RU.md), сборки лежат в *Releases*. **Часть кода написана с помощью ИИ** (Claude Code), собрана и покрыта юнит-тестами, в симуляторе пока проверена мало: сообщайте о найденном. Лицензия GPL v2, как у оригинала, авторы оригинала указаны ниже.

![SSV logo](./Doc/Space%20Shuttle%20Vessel/Images/logo1000.png)

# Space Shuttle Vessel
Space Shuttle Vessel (SSV) is an addon for Orbiter Space Flight Simulator (https://github.com/orbitersim/orbiter). The purpose of this addon is to simulate NASA’s Space Transportation System Program as much as possible. Currently only some elements have been completed and work on others is ongoing.

![Launch](./Doc/Space%20Shuttle%20Vessel/Images/launch.png)
![Upper stage being deployed in orbit](./Doc/Space%20Shuttle%20Vessel/Images/orbit.png)
![Landing viewed from the HUD](./Doc/Space%20Shuttle%20Vessel/Images/landing.png)

This release of SSV is for use in Orbiter 2024.

**This branch is a development version (1.17-dev)** with the PDRS (RMS) and game controller work: see [DEV_BUILD.md](DEV_BUILD.md) (English) or [DEV_BUILD_RU.md](DEV_BUILD_RU.md) (Russian) for what is new, how to test it and the builds.


# Installation
1. Install Orbiter 2024.

2. Install the required addons:

	Antelope Valley scenery pack (https://www.orbiter-forum.com/resources/orbiter-2016-high-resolution-texture-packs.5676/)

3. Extract the SSV files into your Orbiter installation folder, overwriting any existing files.
WARNING: The SSV installation overwrites the default Base.cfg and Earth.cfg files.

4. Install the "SSV_Font_A" and "SSV_Font_B" fonts, located in the "<Orbiter installation>\Install\Space Shuttle Vessel" directory, by opening them and selecting "Install". After successful installation the files can be deleted.

5. The displays in SSV require the MFD resolution of 512 x 512 (Orbiter Launchpad > Extra > Instruments and panels > MFD parameter configuration > MFD texture size).

6. The local light sources have to be enabled for the external lights in SSV to work (Orbiter Launchpad > Visual effects > Local light sources, and additionally in D3D9 Orbiter Launchpad > Video > Advanced > Local lights).

7. For a better visual experience, using the D3D9 graphics client is strongly recommended, although not required. If using the D3D9 graphics client, the "Disable near clip plane compatibility mode" option in the D3D9 Advanced Setup dialog (Orbiter Launchpad > Video > Advanced) should be checked.

NOTE: If you encounter the error "msvcp140.dll is missing" you need to download the Microsoft Visual C++ Redistributable for Visual Studio 2017.


## Optional addons
It is recommended the installation of the excellent Shuttle FDO MFD (https://github.com/indy91/Shuttle-FDO-MFD) by indy91, which performs the calculations done by the Flight Dynamics Officers in Mission Control, allowing an accurate rendezvous profile simulation, as well as targeting a deorbit burn to land at a chosen runway.


## Known issues
A compatibility issue has been identified with the addon "Attachment Manager", so it might have to be disabled for SSV to work properly.


# Credits
Space Shuttle Vessel is based on revision 3242 of Space Shuttle Ultra (svn://orbiter-radio.co.uk/shuttleultra).

The SSV vessels load JSON files with cJSON library by Dave Gamble and cJSON contributors (https://github.com/DaveGamble/cJSON), included in the code. The Mission Editor loads and saves JSON files with Json.NET by James Newton-King (https://www.newtonsoft.com/json).

Large parts of the launch autopilot were copied (with minor modifications) from PEG MFD.

Some of the attitude control code was derived from Attitude MFD V3.

Vandenberg base uses part of the VandenbergAFB-2006 addon (https://www.orbiter-forum.com/resources/vandenbergafb-2006.3523/) by Usonian.

The SurfaceRoving class is based on the GeneralVehicle addon (https://www.orbiter-forum.com/resources/generalvehicle.3158/) by Fred18.

The file "circuit_breaker.wav" comes from Project Apollo - NASSP (https://github.com/orbiternassp/NASSP).


# License
This addon is open-source and is released under the GNU GPL v2 (see file [SSV-LICENSE.txt](SSV-LICENSE.txt) for details).


# Disclaimer
The SSV team is not responsible for any crashes or other problems caused by this addon. Use at your own risk.
