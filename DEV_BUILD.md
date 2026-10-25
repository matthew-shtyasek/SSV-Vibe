# SSV 1.17 development build: PDRS (RMS) and game controllers

This is a **development** version (branch `dev/1.17`), based on SSV v1.16 (Orbiter 2024). It is made for testing, not a release. Nothing here was tested in a long run of the simulator: please report what you find (see the end of this file).

Russian version: [DEV_BUILD_RU.md](DEV_BUILD_RU.md).

## Install

1. Install Orbiter 2024 (x86) and SSV v1.16 as the main README says (fonts, MFD texture size, local lights).
2. Download the zip of the build (the release "Development build", or the artifact of the last run of the "Build" workflow) and extract it in the Orbiter 2024 folder, overwriting existing files.
3. The Mission Editor is not part of this zip (it is a .NET project): keep the one of the v1.16 release.

Run the simulation with the Atlantis or any SSV scenario. Quick start with the arm: scenario **Space Shuttle Vessel > Testing scenarios > Payload Latch Test** (in orbit, the arm holds a test payload), or **Atlantis in orbit**.

## What is new

### Remote Manipulator System (arm), according to the NASA documents
- All the modes of the real arm: **OPR CMD, AUTO 1-4, ORB UNL, END EFF, ORB LD, PL, SINGLE, DIRECT, TEST**. A mode selected with MODE takes effect after **ENTER** (DIRECT does not need it). The point of resolution (POR) of ORB LD, PL and the automatic modes is chosen with the PL ID of SPEC 94.
- The rates follow the deflection of the hand controllers. **RATE HOLD** (Ctrl+Y or a button of the controller) holds the commanded rates, the RATE switch (Ctrl+O) selects coarse or vernier rates.
- **Panel A8U**, with animated parts: BRAKES switch and indicator (the indicator shows the real state of the brakes, with ON also when the arm has no power), **SAFING** with the three positions SAFE, AUTO and CANCEL (momentary): SAFE stops the arm and the safing is maintained until it is cancelled, the indicator shows the barberpole while safing is in progress; AUTO SEQ PROCEED/STOP with the READY and IN PROGRESS lights; the ENTER pushbutton; BACKUP CONTROL (joint selector, DIRECT DRIVE, PAYLOAD RELEASE) and the **RMS POWER** switch of panel A8L (PRIMARY, OFF, BACKUP).
- **Soft stops and reach limits** behave as in the documents: the arm goes idle in the soft stop, SINGLE and DIRECT can drive the joint out of it.
- **Automatic sequences** (AUTO 1-4) are read from `Config\SSV\RMS\AutoSequences.txt` (format in the file). Ctrl+W writes the current position of the POR to `Orbiter.log`, to be pasted in the file.
- **Displays:** SPEC 94 PDRS CONTROL (PL ID, END EFF, I/O, soft stop and auto brake options, auto sequence numbers, OPR CMD end position and attitude with the command check, WR RANGE, ORB LD CMD REF), **DISP 95 PDRS OVERRIDE** (replaces the switches of panel A8U), **DISP 96 PDRS FAULTS** (reach limits) and **DISP 169 PDRS STATUS** (position, attitude, rates, joint angles, end effector state).
- **CTVM monitors:** the menu of the monitor has a new item **OVRLY** (off, on, hint). With the camera RMS (wrist camera) it draws the PDRS target overlay for the grapple (the real one was a plastic sheet put on the monitor); *hint* adds the distance along the camera axis and the angle to the grapple fixture that is in the field of view (a training aid that did not exist in the real vehicle).
- **Payload mass** can slow the arm down (simple model, **off** by default): option `RMSPayloadMassEffect` in `Config\Vessels\SSV_OV.cfg`.
- On the ground (on the launch pad or landed) the payload bay doors, radiators, MPM and the arm cannot be moved (they can not in the real vehicle: they are not designed for 1 g).
- The hand controllers of the shuttle are read at the position of the user (commander, pilot or aft station). The aft THC left-right direction of the keyboard and of the controllers is the same.

### Game controllers (any number)
- Any number of DirectInput devices, independent of the joystick of the Orbiter launchpad (RHC and THC on two sticks, gamepads, FPV radios in USB joystick mode). The configuration files have the format of the Vesim module of NASSP, one file per device (`Config\SSV\Controllers`). **Read `GUIDE_RU.md` (Russian) or `README.txt` there**, with examples for the Thrustmaster T16000M, a gamepad and an FPV radio.
- Axes: RHC pitch, yaw, roll, THC X, Y, Z, **rudder pedals (RPTA), the speedbrake/throttle lever (SBTC), toe brakes**. Buttons: RMS capture, release, rate hold, rate, control toggle, and **any switch or pushbutton of a panel** (`PANEL A8A1/BRAKES:TOGGLE ...`). The operations of the buttons are recorded for playback like the mouse.
- **Ctrl+J** shows the live state of the controllers (raw axes and buttons), Ctrl+Shift+J takes the current position of the axes as their center. **Ctrl+A** toggles the control of the arm with the hand controllers.

### Other
- **UNIV PTG:** TRK (item 19) for the targets Earth fixed point (TGT ID 3: LAT, LON, ALT), Sun (ID 4) and celestial point (ID 5: RA, DEC). The body vector (BODY VECT, P, Y) is pointed at the target, with the roll OM. The orbiting vehicle (ID 1) and the stars are not available.
- **Playback:** the clicks on the panels and the keys that SSV reads are recorded, and replayed (the mouse works during playback).
- **Shuttle FDO MFD** (separate repository, `indy91/Shuttle-FDO-MFD`): the button "Launch from LCC" takes the day and time of liftoff of the countdown of the SSV launch control center (file `Config\SSV\LaunchTime.txt`, written by `SSV_LCC`); a target other than the ISS is chosen when there is no vessel named ISS; fixes of the threads and of the table of instances. These changes are in the repository of the FDO, not here.
- Earth cloud resolution level 11 in `Config\Earth.cfg` (clouds of high resolution, needs the planetary texture archives).

### Tools and the pipeline of this repository
- `tools\build.ps1` builds all the modules (also on a PC with limited resources: one thread, low priority), `tools\get-atl.ps1` gets the ATL libraries without a system installation, `tools\assemble.ps1` puts the build in an Orbiter folder, `tools\package.ps1` makes the zip, `tools\check-encoding.py` reports the encoding of the sources, `tools\get-orbiter-sdk.ps1` gets the SDK from the Orbiter release.
- **Unit tests** (`Orbitersdk\Space Shuttle Vessel\tests`): 8680 checks of the math of the arm, of the UNIV PTG pointing, of the axes, hats and configuration files of the controllers, of the safing and the brakes. `tools\build.ps1 -Orbiter <Orbiter folder> -Tests` runs them, the pipeline too.
- **GitHub Actions** (`.github\workflows\build.yml`): builds, runs the tests and publishes the zip, see below.

## Known issues and limits
- **The visual details of the new switches were set by calculation** from the mesh and the texture and can need adjustment (angles of the switches, position of the new indicators).
- Auto safing, auto brakes, the failures of the arm (ABE, MCIU, tachometers, encoders) and the delays of the brakes and of the safing are not modelled (the documents have no numbers for the delays).
- A master alarm at the start of a playback has been seen and the cause is not known: the line `C&W: parameter N tripped` of `Orbiter.log` tells it. Recordings made with older versions of SSV have no panel events (the mouse is still reset).
- RSLS: the failure branches of the countdown sequence are not implemented (they need data that SSV does not model).
- Everything marked above was built and unit-tested, but not tested in a long run of the simulator.

## Test list
1. Start **Payload Latch Test**. Look at panel A8U (aft flight deck, RMS station): the BRAKES switch and its indicator, SAFING (three positions, momentary CANCEL), AUTO SEQ, MODE and ENTER.
2. Release the brakes, select END EFF and press ENTER, press Ctrl+A and move the arm with the keyboard or the sticks; the rates must follow the deflection. Capture and release the payload with the trigger (Ctrl+Enter, Ctrl+Backspace).
3. SPEC 94 (item 3 = PL ID after the capture), OPR CMD: end position and attitude, item 25 (command check GOOD), then the MODE switch to OPR CMD, ENTER and AUTO SEQ to PROCEED.
4. Drive a joint into its soft stop, then leave it with SINGLE. Switch SAFING to SAFE, back to AUTO (still safed) and CANCEL.
5. RMS POWER to BACKUP: the backup drive with the joint selector and DIRECT DRIVE. DISP 95, 96 and 169.
6. CTVM: the camera RMS on the monitor, the menu of the monitor, OVRLY on and hint, near a grapple fixture (for example the OBSS of the scenario "STS-126 docking with ISS Nov 16").
7. Two sticks: start a simulation with the devices connected, read `Config\SSV\Controllers\DetectedDevices.txt`, copy the examples (`Joystick1.cfg`, `Joystick2.cfg`), press Ctrl+J.
8. Record a short part of the flight and play it back (the arm and the panel clicks must be replayed).
9. UNIV PTG: TGT ID 4, then TRK.

## Report a problem
Open an issue with: the scenario, what you did, and `Orbiter.log` (and `Config\SSV\Controllers\DetectedDevices.txt` for the controllers).

## Build and the pipeline
Locally (Visual Studio 2022 or 2026 with C++ and ATL, Orbiter 2024 folder for the SDK):

```
powershell -File tools\build.ps1 -Orbiter "C:\Games\Orbiter 2024" -Toolset v143 -Tests
powershell -File tools\package.ps1 -Version v1.17-dev.1
```

The workflow `Build` runs on every push and pull request (build, tests, zip as an artifact of the run). A push to a branch `dev/*` also updates the pre-release **dev-latest** with the zip, and a tag `v*` (for example `v1.17-dev.1`) makes a release.
