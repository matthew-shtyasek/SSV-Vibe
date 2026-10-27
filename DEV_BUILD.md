# SSV 1.17 development build: PDRS (RMS) and game controllers

This is a **development** version (branch `dev/1.17`), based on SSV v1.16 (Orbiter 2024). It is made for testing, not a release. Nothing here was tested in a long run of the simulator: please report what you find (see the end of this file).

## Build 1.17-dev.3

Install: extract the zip in the Orbiter 2024 folder (x86), overwriting existing files (keep the Mission Editor of the v1.16 release; it is not in the zip).

**Most of it is not validated in the simulator: please send `Orbiter.log` with the problems.**

New since dev.2:
- **UNIV PTG** is the full software of indy91 (https://github.com/indy91/SSV, branch experimental): all targets (orbital object, Earth center, Earth point, Sun, celestial, 100 navigation stars), star catalog, DAP A01-A15/B01-B15. Tracking an orbital object (TGT ID 1) needs a target with REL NAV on; without it the attitude is held. The ADI of the ORBIT PFD follows the ADI ATTITUDE switch (INRTL and REF: the inertial attitude of UNIV PTG; LVLH: the horizon).
- **Settings MFD** (mode SSV Settings, key V; P turns the pages): page CONTROLLERS (assign the axes and buttons by moving or pressing them, reverse, clear, center, save to `Config\SSV\Controllers`).
- **Game controllers:** a configuration file can name its device with the line `DEVICE<tab>part of the name`; files are given to the controllers by name first and by order among controllers of the same name, so the order that Windows gives does not matter. A stick axis that is not at its center at the first reading is ignored (a pedal that took the configuration of a stick no longer rotates the orbiter). See `Config\SSV\Controllers\README.txt`.
- **Hydraulics:** HYD CIRC PUMP 1-3 (panel R2, GPC logic of the Crew Operations Manual), heaters and freezing of the boiler lines in the Earth's shadow, nitrogen use, random failures of the boilers (option `HydraulicRandomFailures`).
- **Lighting:** EMERGENCY LIGHTING switch on C3; floodlights of the middeck (option `MiddeckLights`); the panels follow the brightness of the cabin (option `PanelGlow`).
- **RCS and time acceleration:** the DAPs reduce the thrust only above a time acceleration (option `RCSFullThrustAcc`, 10x by default), the flames are drawn with time acceleration.
- **RMS:** the inertia of the links of the arm in the moments of inertia of the orbiter (options `RMSInertia` and `RMSCenterOfMass`, off by default).

Credits: Niklas Beug (indy91) for the Universal Pointing and the work on the ascent guidance in the base; see `Doc\Space Shuttle Vessel\CREDITS.tex`.

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

### RMS control through the GPC (PDRS software)

As in the real vehicle, the computer supported modes of the arm do not run inside the arm: they run in the software of the SM GPC (SM OPS 2, major modes 201 and 202).

- **How the data goes.** The MCIU (the interface unit of the arm) puts a frame on the data bus every cycle: the hand controllers, the switches of panel A8U (MODE, ENTER, joint select, RATE, AUTO SEQ, SAFING, RMS POWER), the angles of the joints, the end effector talkbacks and the payload that the arm holds. The PDRS software runs at 12.5 Hz (80 ms), computes the commands of rate of the six joints and returns them. The MCIU applies them to the joints. Because of this loop the motion lags the hand controller by about one software cycle.
- **What runs in the software:** SINGLE, ORB UNL, END EFF, ORB LD, PL, TEST, OPR CMD, AUTO 1 to 4, RATE HOLD, the soft stops, the points of resolution (POR), the DISP 95 overrides and the displays SPEC 94 PDRS CONTROL, DISP 95 PDRS OVERRIDE, DISP 96 PDRS FAULTS and DISP 169 PDRS STATUS (they show the data of the software).
- **What does not use the computer (hardware):** DIRECT drive, the BACKUP control (RMS POWER in BACKUP and the BACKUP CONTROL switches), the brakes and the SAFING. Therefore, **if the GPC does not run the software (major mode other than 2xx, or the SM GPC is not working), the arm cannot be driven with the hand controllers: use DIRECT or BACKUP.**
- **Loss of the software:** if the software does not answer for 0.5 s the MCIU holds the commands at zero and the arm stops (the mode lights go out until the software answers again).
- **ENTER:** the mode selected with MODE is taken only when ENTER is pushed (as before); DIRECT does not need it.
- **When the software runs:** it is active when the SM GPC is in MM 201 or 202. Until it has run once in a session (for example a scenario that starts in another major mode), the old direct control is used. After that, leaving MM 201/202 takes the computer supported modes away, as in the real vehicle.
- **I/O:** SPEC 94 item 5 (I/O ON) and item 6 (I/O OFF). With I/O OFF the software does not command the arm (only DIRECT and BACKUP work). The state is kept in the scenario (`PDRS_IO`); a scenario without it starts with I/O ON. A scenario that starts with the arm in use should also have the circuit breakers DDU LEFT and DDU RIGHT closed, otherwise the hand controllers of the commander and the pilot have no power (only the aft one works).
- **Option** `RMSViaGPC` (on by default; Settings MFD, group RMS). Off gives the old direct control, which works in any major mode. In the game: **Ctrl+Shift+A** switches it on and off (the label tells the state), or the Settings MFD.
- The scenario keeps `ACCEPTED_MODE` and `PDRS_ID`, the other data of SPEC 94 start from the defaults.
- **Not validated in the simulator yet:** the behaviour is covered by unit tests of the logic (the closed loop of the modes, the frames of the bus) but nobody has flown it. Please report the lag, the modes that do not engage, and the cases where the arm does not move.

### Game controllers (any number)
- Any number of DirectInput devices, independent of the joystick of the Orbiter launchpad (RHC and THC on two sticks, gamepads, FPV radios in USB joystick mode). The configuration files have the format of the Vesim module of NASSP, one file per device (`Config\SSV\Controllers`). **Read `GUIDE_RU.md` (Russian) or `README.txt` there**, with examples for the Thrustmaster T16000M, a gamepad and an FPV radio.
- Axes: RHC pitch, yaw, roll, THC X, Y, Z, **rudder pedals (RPTA), the speedbrake/throttle lever (SBTC), toe brakes**. Buttons: RMS capture, release, rate hold, rate, control toggle, and **any switch or pushbutton of a panel** (`PANEL A8A1/BRAKES:TOGGLE ...`). The operations of the buttons are recorded for playback like the mouse.
- **Ctrl+J** shows the live state of the controllers (raw axes and buttons), Ctrl+Shift+J takes the current position of the axes as their center. **Ctrl+A** toggles the control of the arm with the hand controllers.

### Other
- **UNIV PTG:** TRK (item 19) for the targets Earth fixed point (TGT ID 3: LAT, LON, ALT), Sun (ID 4) and celestial point (ID 5: RA, DEC). The body vector (BODY VECT, P, Y) is pointed at the target, with the roll OM. The orbiting vehicle (ID 1) and the stars are not available.
- **Playback:** the clicks on the panels and the keys that SSV reads are recorded, and replayed (the mouse works during playback).
- **Shuttle FDO MFD** (separate repository, `indy91/Shuttle-FDO-MFD`): the button "Launch from LCC" takes the day and time of liftoff of the countdown of the SSV launch control center (file `Config\SSV\LaunchTime.txt`, written by `SSV_LCC`); a target other than the ISS is chosen when there is no vessel named ISS; fixes of the threads and of the table of instances. These changes are in the repository of the FDO, not here.
- Earth cloud resolution level 11 in `Config\Earth.cfg` (clouds of high resolution, needs the planetary texture archives).

### APU (shutdown, cooldown, restart)
- **APU OPERATE** has the three positions of the real switch: START/RUN, OFF and INJECTOR COOL (before, INJECTOR COOL started the APU).
- **Auto shutdown:** the switches APU AUTO SHUT DOWN (ENABLE/INHIBIT) work. With ENABLE the controller shuts the APU down below 80% (only 10.5 s after the start command) and above 129% of the speed, and closes the fuel tank isolation valves. The outputs for the UNDERSPEED and OVERSPEED lights of panel F7 are in the bundle `R2_To_APUn` (lines 9 and 10), the lights themselves are not connected to the C&W logic yet. **After an auto shutdown the FUEL TK VLV switch has to go to CLOSE before the APU OPERATE switch goes to OFF**; if it is left in OPEN the valves open again and the fuel reaches the (hot) gas generator.
- **APU SPEED SELECT** (NORM 103%, HIGH 113%).
- **Restart after a shutdown:** the injector branch is hot after the shutdown (soakback) and needs **about 180 minutes** to get to 400 F, or **3.5 minutes of water cooling** (APU OPERATE in INJECTOR COOL; do not wait more than 1 s in OFF before START/RUN, the injector heats up again quickly). The water tank is common to the three APUs: 21 minutes of flow, six cycles. Starting an APU above the limits (injector 400 F, fuel pump 210 F, gas generator valve module 200 F) can detonate the hydrazine and **destroy the APU**; so can a restart after an overspeed shutdown.
- For tests: the key `Failure 1` (the speed falls) or `Failure 2` (the speed rises) in the block of the subsystem `APU1` of a scenario makes the speed of a running APU leave the limits, to see the auto shutdown.
- The temperatures are in the bundle `APU_n_SENSORS` (lines 1, 2, 3 for the injector, the pump and the valves, 4 for the lube oil); the temperature of the lube oil goes to the displays, the others do not.
- Fix of an upstream bug: `Atlantis::GetAPU` used the wrong index (SPEC 113 read the wrong objects).

### The mass of the arm and of its payload: four separate options
All are in `Config\Vessels\SSV_OV.cfg` (and in the settings MFD), **off by default**, and independent:
- `RMSPayloadMassEffect`: the rates of the arm decrease with the mass of the payload that it holds (it is about the behavior of the arm).
- `RMSReaction`: the arm and the payload that it moves push the orbiter (force at the center of mass of the payload and of the arm, mass times the acceleration relative to the orbiter), so the orbiter turns when the DAP is in FREE (try it with a heavy payload).
- `RMSCenterOfMass`: the center of mass of the orbiter follows the arm and the payload that it holds (the arm is 410.5 kg, in five points along it), so the RCS jets act about the real center of mass.
- `RMSInertia`: the moments of inertia of the orbiter change with the arm and the payload (parallel axes). With the center of mass option on, the moments are those about the new center.
These were made from the geometry of the arm and the mass of the payload; they were not tested in the simulator, and the distribution of the mass of the arm along its links is an assumption.

### Cabin lighting (Crew Operations Manual 2.15)
- Floodlights (light emitters in the cockpit views; Orbiter needs the local lights on): glareshields of the commander and the pilot (switch BRIGHT, VAR, OFF and DIM/BRT knob, panels O6 and O8), seat and center console lights (SEAT, OFF, CTR CNSL and knob), mission station (R10), payload station (L9) and orbit station (A6U) with ON/OFF and DIM/BRT.
- Panel lighting: the OFF/BRT knobs of O6 (LEFT/CENTER and LEFT OVERHEAD), O8 (RIGHT/CENTER and RIGHT OVERHEAD), R10 (MS) and A6U (OS) scale the self illumination of the panels of their groups (the knobs at BRT leave the panels as they were).
- Not connected yet: the knobs INSTRUMENT, NUMERIC and ANNUNCIATOR INTENSITY, the circuit breakers of the lighting, the emergency lighting, the middeck and airlock lights. Option `CabinLighting`.

The pointers of the DIM-BRT and OFF-BRT knobs are calibrated live in the Settings MFD (group Cabin): for each group of panels (O6/O8, A6U, L9, R10) `KnobXxxOffset` (angle of the pointer of the mesh at rest from the middle of the travel, deg, counterclockwise positive), `KnobXxxTravel` (rotation between DIM/OFF and BRT, deg) and `KnobXxxReverse`. The knobs follow the changes at once. Report the values that match the labels; they will become the defaults.

### Free movement of the head
Ctrl+Home (the key is the option `VCFreeMoveKey`, chosen in the Settings MFD: Home, End, Page Up, Page Down, Space; Ctrl+M is the map of Orbiter) switches it on (and off); then the arrows and Ins/Del move the head (forward, back, left, right, up, down) around the position of the station, inside an ellipsoid (0.7 m to the sides and forward/back, 0.35 m up, 0.7 m down). The arrows and Ins/Del do not operate the THC while it is on. It ends by itself when the view changes to another station. Option `VCFreeMove`.

### Settings MFD
The MFD mode **SSV Settings** (key V in the list of modes, or from the MFD menu) shows the options of the vessel and changes them at once; the SAVE button writes `Config\SSV\Settings.cfg` (it has the format of the configuration of the vessel and replaces the values of `SSV_OV.cfg` when the next simulation starts). Page 2 sets failures of the water spray boilers (training), page 3 shows the temperatures, water and reservoir of the three hydraulic systems. Keys: Up/Down, Left/Right (or the buttons), Enter, S (save), Del (default), A (all), P (page).

### Hydraulic fluid and APU lube oil: heating, cooling and failures
- Each APU/hydraulic system has a **water spray boiler** (WSB; panel R2: BOILER PWR, CNTLR/HTR A or B, N2 SUPPLY) with the data of the Crew Operations Manual: spray of the lube oil from 250 F, of the hydraulic fluid from 208 F, bypass valve (through the boiler from 210 F, around it from 190 F), 142 lb of water (spray bars of 10 and 5 lb/min) pushed by nitrogen.
- The lube oil of a running APU heats (about 15 minutes from cold to 250 F) and the hydraulic fluid heats with the load (aerosurfaces, engine gimbals) and the pump pressure (LOW gives less heat).
- **Overheating:** more than 230 F in the fluid is the caution HYD RSVR T, more than 290 F in the oil is APU TEMP (both in `Orbiter.log`; the APU TEMP light of the C&W is lit by the same limits, the message of the SM is not done yet); **a minute above 290 F and the bearings of the APU seize** (the APU is lost), above 250 F in the fluid the seals degrade and the system leaks until it loses the pressure. As in the manual, change the boiler controller (A or B) first.
- Failures (page 2 of the MFD, or in the scenario, block `WSB1` to `WSB3`: `FailCtrlA`, `FailCtrlB`, `IceHyd`, `IceOil`, `N2Leak`, `HydLeak`).
- The OIL IN temperatures, the H2O quantities and the reservoir quantities now go to the displays (DISP 86 and MEDS APU/HYD) through the analog inputs. Option `HydraulicThermal` (off: the temperatures do not change). The scenarios that start with a running APU begin with a warm system.

### Controls of the hand controllers by station
The hand controllers and the keys operate the RMS when the view is the aft station of the RMS, and the RCS in every other place (before, Ctrl+A did it for the whole cockpit). Ctrl+A, or the controller button RMS_CONTROL_TOGGLE, inverts it until the station is changed.

### Tools and the pipeline of this repository
- `tools\build.ps1` builds all the modules (also on a PC with limited resources: one thread, low priority), `tools\get-atl.ps1` gets the ATL libraries without a system installation, `tools\assemble.ps1` puts the build in an Orbiter folder, `tools\package.ps1` makes the zip, `tools\check-encoding.py` reports the encoding of the sources, `tools\get-orbiter-sdk.ps1` gets the SDK from the Orbiter release.
- **Unit tests** (`Orbitersdk\Space Shuttle Vessel\tests`): 43000 checks of the math of the arm, of the UNIV PTG pointing, of the axes, hats, parsers and example configuration files of the controllers, of the safing and the brakes, of the APU (cooldown, restart, auto shutdown), of the reaction, mass and inertia of the arm, of the cabin lighting (levels and the table of the panels), of the free movement of the head, of the table of the options (checked against `SSV_OV.cfg`) and of the boiler (heating, cooling, failures, times of the manual). `tools\check-data.py` checks the meshes against the group indices of the code and the scenarios (a wrong index shows as a switch or a light that does not work) and the table of the panels of the cabin lighting. `tools\build.ps1 -Orbiter <Orbiter folder> -Tests` runs them, the pipeline too.
- **GitHub Actions** (`.github\workflows\build.yml`): builds, runs the tests and publishes the zip, see below.

## Known issues and limits
- **The visual details of the new switches were set by calculation** from the mesh and the texture and can need adjustment (angles of the switches, position of the new indicators).
- The new visuals (knobs and switches of the lighting, light positions, head movement, the MFD) and the physics of the options of the arm and of the boilers were set by calculation and are not tested in the simulator: the angles, the places of the lights and the heating rates may need adjustment.
- The lights APU TEMP, UNDERSPEED and OVERSPEED of the C&W are connected to the model; the light HYD RSVR T and the SM messages are not.
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
10. Cabin lighting: O6, DIM/BRT knobs (hold the left button on the right or left half of the knob), flood switches; turn the panel knobs down and up.
11. Ctrl+Home and the arrows in the stations; at the aft RMS station the controllers move the arm.
12. MFD SSV Settings: change an option, SAVE, restart the scenario.
13. APU start with the boilers off (BOILER PWR OFF cuts the READY TO START): the boiler failures of the MFD with a running APU, watch `Orbiter.log` and DISP 86.

## Report a problem
Open an issue with: the scenario, what you did, and `Orbiter.log` (and `Config\SSV\Controllers\DetectedDevices.txt` for the controllers).

## Build and the pipeline
Locally (Visual Studio 2022 or 2026 with C++ and ATL, Orbiter 2024 folder for the SDK):

```
powershell -File tools\build.ps1 -Orbiter "C:\Games\Orbiter 2024" -Toolset v143 -Tests
powershell -File tools\package.ps1 -Version v1.17-dev.1
```

The workflow `Build` runs on every push and pull request (build, tests, zip as an artifact of the run). A push to a branch `dev/*` also updates the pre-release **dev-latest** with the zip, and a tag `v*` (for example `v1.17-dev.1`) makes a release.
