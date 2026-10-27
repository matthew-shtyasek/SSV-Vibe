/****************************************************************************
  This file is part of Space Shuttle Vessel

  Hand controller input (RHC and THC) from game controllers


  Space Shuttle Vessel is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  Space Shuttle Vessel is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Space Shuttle Vessel; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

  See https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html or
  file SSV-LICENSE.txt for more details.

  **************************************************************************/
#ifndef _HANDCONTROLLERS_H_
#define _HANDCONTROLLERS_H_

#include <string>


/**
 * Reads any number of game controllers (joysticks, gamepads, FPV radios in USB joystick mode) with DirectInput, and connects their
 * axes and buttons to the inputs of the hand controllers.
 * The configuration is read from one file per controller in Config\SSV\Controllers (see the README.txt in that folder), which is
 * the same approach used by the Vesim module of NASSP. This allows, for example, one controller to be the RHC and another one the THC,
 * or the axes of the RHC and THC to be taken from different controllers.
 * Inputs without a connection are not changed, so the keyboard and the Orbiter joystick keep working for them.
 */
class HandControllers
{
public:
	enum Input
	{
		// axes: -1 to +1
		RHC_PITCH = 0,// +1 = pitch up
		RHC_YAW,// +1 = yaw right
		RHC_ROLL,// +1 = roll right
		THC_X,// +1 = back (-X)
		THC_Y,// +1 = right
		THC_Z,// +1 = up
		RPTA,// rudder pedals: +1 = right pedal forward (yaw right)
		SBTC,// speedbrake/throttle controller (commander or pilot): +1 = lever fully forward, -1 = fully aft
		BRAKE_LEFT,// left toe brake: +1 = fully pressed, -1 = released (the range of the axis is used, set DEADZONE to 0)
		BRAKE_RIGHT,// right toe brake: same as the left
		// buttons
		RMS_CAPTURE,// RHC trigger (lower half)
		RMS_RELEASE,// RHC trigger (upper half)
		RMS_RATE_HOLD,// RHC RATE HOLD pushbutton
		RMS_RATE_TOGGLE,// RHC RATE switch (toggles between vernier and coarse)
		RMS_CONTROL_TOGGLE,// toggles the control of the RMS with the hand controllers
		INPUT_COUNT
	};

	/** Actions of a button on a switch or pushbutton of a panel. */
	enum PanelAction {PA_UP = 0, PA_DOWN, PA_MID, PA_TOGGLE, PA_HOLD_UP, PA_HOLD_DOWN, PA_PRESS};

	struct PanelEvent
	{
		std::string panel;
		std::string component;
		int action;
		bool pressed;// button pressed (true) or released (false)
	};

	HandControllers( void );
	~HandControllers( void );

	/**
	 * Finds the controllers and reads the configuration files. To be called once the simulation is running.
	 */
	void Init( void );

	/**
	 * Reads the state of the controllers. To be called each time step.
	 */
	void Poll( void );

	/**
	 * @return true if the input is connected to at least one axis or button
	 */
	bool HasInput( int input ) const;

	/**
	 * @return value of an axis input, -1 to +1 (0 if the input is not connected)
	 */
	double GetAxis( int input ) const;

	/**
	 * @return state of a button input
	 */
	bool GetButton( int input ) const;

	/**
	 * Gets the next change of a button that is connected to a panel component (configuration line "PANEL <panel>/<component>:<action>").
	 * @return false if there is none
	 */
	bool PopPanelEvent( PanelEvent& e );

	/**
	 * Takes the current position of the axes that are connected to the inputs as their center (for controllers that do not return to the center).
	 * The values are also written to the log, to be copied to the configuration file.
	 */
	void CalibrateCenters( void );

	/**
	 * Text with the state of the controllers and of the inputs, for the live view.
	 */
	std::string GetStatusText( void ) const;

	// ---- used by the controllers page of the settings MFD ----

	/** @return true if the input is an axis (-1 to +1), false if it is a button */
	static bool IsAxisInput( int input ) { return input < RMS_CAPTURE; }

	/** @return name of the input as written in the configuration files */
	static const char* InputName( int input );

	int DeviceCount( void ) const;
	std::string DeviceName( int index ) const;

	/** While on, the controllers are read even if none is configured (to assign them). */
	void SetRawPolling( bool on );

	/** @return the connections of the input, for example "J1 Axis X, J2 Button 3 (rev)", "-" if there are none */
	std::string DescribeInput( int input ) const;

	/** Takes the current position of all controllers as the position at rest, for LearnStep. */
	void BeginLearn( void );

	/**
	 * Looks for an axis that moved (axis inputs) or a button/hat direction that was pressed (button inputs) since BeginLearn and, if found,
	 * connects it to the input in place of the connections it had.
	 * @return true if the input was assigned
	 */
	bool LearnStep( int input );

	/** Waits (checked in each Poll) for the next movement of an axis or press of a button, and assigns it to the input. */
	void StartLearn( int input );
	void StopLearn( void );
	/** @return the input that waits for an assignment, -1 if none */
	int LearningInput( void ) const;

	/** Removes the connections of an input. */
	void ClearInput( int input );

	/** Reverses the direction of the connections of an input. @return false if there are none */
	bool ToggleReverse( int input );

	/**
	 * Writes a configuration file for each controller that has connections (or already had a file); the previous file is kept as .bak.
	 * @param message result, for the display
	 */
	bool SaveConfigs( std::string& message );

	struct Impl;// implementation (public so the DirectInput callbacks can use it)

private:
	Impl* impl;
};

#endif// _HANDCONTROLLERS_H_
