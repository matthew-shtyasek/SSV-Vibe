# SSV 1.17-dev.4 (pre-release): what changed since dev.3

Development version based on SSV v1.16 (Orbiter 2024). It is made for testing, not a release. Please report problems as an issue: the scenario, what you did and `Orbiter.log`.

## Hydraulics and the aerosurfaces
- **Actuators by system:** every elevon has a switching valve with three sources (as in DISP 87), 20 deg/s and 13.9 deg/s on a single system; the rudder/speedbrake and the body flap have three motors, one per system; brakes use systems 1 and 2, system 3 takes over; nose wheel steering. With no pressure the elevons float with the flow.
- **Reservoir quantity:** starts at 60% and follows the thermal expansion of the fluid with its temperature (about 0.12% per degree F).
- **Heating and cooling:** fluid and APU lube oil heat up, the water spray boilers cool them, the failures from overheating (option `HydraulicThermal`).
- **Realism page** in the SSV Settings MFD: tolerances of the parts (fixed for the vehicle, the seed is saved) and noise of each use, with a scale of all the dispersions.

## APU failures
- **Failures with effects:** the chance of a detonation at a hot start grows smoothly with the temperature (`1-exp(-excess/150 F)`, 0.95 at most); a destroyed APU (explosion or fire) leaks hydrazine while its FUEL TK VLV switch is in OPEN; an explosion destroys another APU with a chance of 50%, a leak of one minute burns another one with 15%; the hydraulic system of a destroyed APU can lose its fluid (50% after an explosion, 20% after a fire).
- **Seen from outside:** the flash of an explosion with a muffled bang (heard outside only), the fire, the dark smoke that rises, and the white vapour of a leak, at the aft compartment doors on the side of the APU (APU 1 and 2 port, APU 3 starboard; the places are approximate). A leak burns only in the air; the option **Random APU fire (like STS-9)** makes a working APU leak and burn at random, to train.
- **Lights and displays:** the C&W lights APU TEMP, UNDERSPEED and OVERSPEED stay on after an auto shutdown until the APU OPERATE switch goes to OFF; the temperatures of the injector, the pump and the valve module are in the SSV HYDRAULIC STATUS page of the MFD and in the new DISP 86 APU/HYD page (SM, OPS 2).
- **Restart as in flight:** an APU that shut down on underspeed can be cooled (INJECTOR COOL, at least 3.5 minutes) and restarted; after an overspeed the restart destroys it.
- **Scenario keys:** `Failure 1` (the speed falls) and `Failure 2` (rises) end with the shutdown of the APU; `Fire 1` makes the leak burn from the start.
- **Training scenarios** (folder Training scenarios): underspeed, overspeed, an explosion with a leak, a fire, a hot restart, a leak of the hydraulic fluid, two APUs lost like STS-9.

## RCS and the DAPs
- **44 real jets** (38 primary and 6 vernier) of the Orbiter Data Book instead of the idealized thrusters: the jets of each rotation and translation are selected from their geometry, every jet has a valve (delays, a minimum on-time of 80 ms) and its own propellant use; jets that fail (`RCS_FAILED` in the scenario) are replaced by others.
- **Time acceleration:** the DAPs divide the thrust by the acceleration, the flames are drawn; from the acceleration of the option `RCSRailsAbove` (300x by default) the orbit DAP holds the attitude on rails, without the jets, and takes the mean vernier propellant use from the tanks.
- **Vernier mode** of the orbit DAP with the true torque of the vernier jets; in real time the rate limits are not finer than the impulse of the shortest firing of a jet.

## Displays
- **ADI** works in all the major modes (the Orbit PFD showed a fixed ball before MM 200); the A/E PFD in MM 104-106 and 301-303 follows the switch ADI ATTITUDE and shows the attitude errors of the OMS burn.

## RMS and the controllers
- The reaction of the arm no longer pushes the orbiter at the start (a jump of the position was taken for a velocity).
- In an external view the hand controllers go where they went in the last station: the RMS if it was the aft station of the RMS, the RCS otherwise; Ctrl+A switches, also in the external view.
- Fine control of the arm with Shift (keyboard only); a CRT driven by the BFS shows no SM display either.

## Flight logs and settings
- **Option `FlightDataLog`** (on by default): files in `SSV_FlightLogs` with the DAP (profiles, mode, target, payload, mass, inertia, c.g., errors, rates, propellant, firings), the hydraulics (`*_hyd_*.log`: APU, surfaces, brakes; lines RESP with the command, position and rate of every surface while they move) and the rotation (`*_rot_*.log`).
- The SSV Settings MFD keeps its page, the selected line and the unsaved mark when the window changes its size.

## Other
- A speed out of the range of a running APU in an old scenario is set to 100% (the APU no longer shuts down at the start).
- Earth cloud resolution level back to 9 in `Config\Earth.cfg` (level 11 breaks the clouds without a cloud archive of that level).
