Game controllers for the hand controllers (RHC and THC) of SSV
=================================================================

SSV can read any number of game controllers (joysticks, gamepads, flight sticks, FPV radios in USB joystick mode) directly with
DirectInput, independently of the joystick that is selected in the Orbiter Launchpad. This makes it possible to use one controller
as the RHC (rotation) and another one as the THC (translation) at the same time, or to take only some of the axes from a controller.
The approach and the format of the configuration files are the ones of the Vesim module of NASSP.

As in the real vehicle, the rates of the RMS are proportional to the deflection of the hand controllers.

How to set it up
----------------
1. Connect the controllers and start a simulation with SSV. The file "DetectedDevices.txt" in this folder lists the controllers found,
   with their names, number (Joystick1, Joystick2, ...) and axes.
2. For each controller, copy one of the .cfg.example files of this folder to "<controller name>.cfg" (name exactly as in the
   list) or to "Joystick<number>.cfg" (useful if you have two controllers with the same name), and edit it.
   A controller without configuration file is ignored.
3. Restart the simulation. The log of Orbiter (Orbiter.log) shows how many connections were read for each controller.
4. If the Orbiter joystick is also selected in the Orbiter Launchpad, the inputs that are not configured here (and the keyboard) keep
   working, but the same stick would be read twice. Disable the Orbiter joystick if you use the same one here.

Format of the configuration files
---------------------------------
One line per connection, columns separated by tabs (commas also work); "#" starts a comment:

   INPUT    SUBDEVID    REVERSE    DEADZONE    EXPO    CENTER

INPUT       RHC_PITCH, RHC_YAW, RHC_ROLL        rotation axes (+1 = pitch up, yaw right, roll right)
            THC_X, THC_Y, THC_Z                 translation axes (+1 = back, right, up)
            RPTA                                rudder pedals (+1 = right pedal forward). Analog; only for the commander or the pilot
            SBTC                                speedbrake and thrust controller lever of the commander or the pilot (+1 = fully forward).
                                                Uses the whole range of the axis (the position of the lever is the position of the axis)
            BRAKE_LEFT, BRAKE_RIGHT             toe brakes (pressed = high end of the axis; use REVERSE if needed and DEADZONE 0)
            RMS_CAPTURE, RMS_RELEASE            trigger of the RHC (capture = lower half, release = upper half)
            RMS_RATE_HOLD                       RATE HOLD pushbutton of the RHC
            RMS_RATE_TOGGLE                     RATE switch of the RHC (toggles vernier / coarse)
            RMS_CONTROL_TOGGLE                  toggles the control of the RMS with the hand controllers (Ctrl+A)
SUBDEVID    Axis X, Axis Y, Axis Z, Axis RX, Axis RY, Axis RZ, Slider 0, Slider 1, Button 0 ... Button 127, POV 0H, POV 0V (hat 0,
            horizontal / vertical; same for hats 1 to 3)
REVERSE     true to invert the axis. For a button input from an axis, "true" uses the negative half of the axis
DEADZONE    dead zone around the center (detent of the hand controller), 0 to 0.9, default 0.05
EXPO        0 = linear (default) to 1 = cubic response (finer control near the center)
CENTER      axis value (-1 to +1) that is taken as zero, default 0 (for controls that do not return to the center)

PANEL <panel>/<component>:<action>    a button that operates a switch or pushbutton of a panel, for example
            PANEL A8A1/BRAKES:UP        PANEL A8A1/AUTO SEQ:TOGGLE        PANEL A8A1/ENTER:PRESS
            panel and component names are the ones used in the scenario files (case does not matter). Actions:
            UP, DOWN, MID, TOGGLE   move a switch (maintained) when the button is pressed
            HOLD_UP, HOLD_DOWN      spring loaded switches: operated while the button is pressed
            PRESS                   pushbuttons: pressed while the button is pressed
            These operations are also recorded in the flight recorder, like the clicks of the mouse.

Several lines can use the same INPUT (for example the same button on two controllers); the one with the larger deflection is used.
An axis can drive a button input (the button is pressed when the axis is beyond 70%), which allows the switches of a radio to be used.

Inputs that are not configured are not changed, so the keyboard and the Orbiter joystick still work for them.
In the orbiter, the hand controllers are read where the user is (commander, pilot or aft station), as with the keyboard.

Live view and calibration
-------------------------
Ctrl+J shows the state of the controllers on the screen: raw axes, pressed buttons and the values of the connected inputs. Use it to find
the axes and buttons of a controller. Ctrl+Shift+J takes the current position of all connected axes as their center and writes the values to
Orbiter.log, to be copied to the CENTER column (useful for the throttle stick of a radio, which does not return to the center).

FPV radios (for example the BetaFPV LiteRadio 3)
-------------------------------------------------
Radios in USB joystick mode appear as a game controller with up to 8 axes (the 4 sticks and the other channels, which include the
switches and knobs). See FPV_Radio.cfg.example. Notes:
- The radio must be in the USB joystick mode (in EdgeTX / OpenTX: SYS > USB mode > Joystick (HID)); the channel order depends on the
  radio setup, so check the axes in the "Game Controllers" panel of Windows and adjust the file.
- The throttle stick does not return to the center. It is useful as a THC axis, because it keeps its position (like RATE HOLD), but set
  CENTER to the position where you want zero (or use the stick centered in the middle of its travel).
- 2 and 3-position switches are axes. Use the axis as a button input (for example RMS_CAPTURE on the up position, RMS_RELEASE with
  REVERSE true on the down position).
