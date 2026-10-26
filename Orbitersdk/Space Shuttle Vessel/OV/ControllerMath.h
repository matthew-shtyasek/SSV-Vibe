/****************************************************************************
  This file is part of Space Shuttle Vessel

  Math and text helpers of the game controller input (no DirectInput here, so they can be tested alone)


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
#ifndef _CONTROLLERMATH_H_
#define _CONTROLLERMATH_H_


#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>


namespace ctrlmath
{
	inline double Clamp( double v )
	{
		return (std::max)( -1.0, (std::min)( 1.0, v ) );
	}

	/**
	 * Value of an axis of a game controller after the center, the direction, the dead zone and the response curve.
	 * @param raw value of the axis as read by DirectInput (-1000 to +1000)
	 * @param center value (-1 to +1) that is taken as zero
	 * @param reverse true to invert the axis
	 * @param deadzone dead zone around the center (0 to 0.9), the rest of the range is rescaled to the full range
	 * @param expo 0 = linear, 1 = cubic
	 * @return -1 to +1
	 */
	inline double ShapeAxis( double raw, double center, bool reverse, double deadzone, double expo )
	{
		double v = Clamp( (raw / 1000.0) - center );
		if (reverse) v = -v;
		double a = fabs( v );
		if (a < deadzone) return 0.0;
		a = (a - deadzone) / (1.0 - deadzone);
		a = ((1.0 - expo) * a) + (expo * a * a * a);
		return (v < 0.0) ? -a : a;
	}

	/**
	 * Horizontal and vertical direction of a hat switch.
	 * @param pov value of the hat in hundredths of a degree (0 = up, clockwise), 0xFFFF in the low word when centered
	 * @param[out] h +1 = right, -1 = left
	 * @param[out] vert +1 = down, -1 = up
	 * @return false if the hat is centered
	 */
	inline bool PovToDirection( unsigned long pov, double& h, double& vert )
	{
		h = 0.0;
		vert = 0.0;
		if ((pov & 0xFFFF) == 0xFFFF) return false;
		const double angle = pov * 3.14159265358979 / 18000.0;
		const double x = sin( angle );
		const double y = -cos( angle );
		const double norm = (std::max)( fabs( x ), fabs( y ) );
		if (norm < 1e-6) return false;
		h = x / norm;
		vert = y / norm;
		return true;
	}

	/**
	 * Position of a lever (0 to 1) from a bipolar axis (-1 to +1), for the levers and pedals that use the whole range of the axis.
	 */
	inline double UnipolarFromAxis( double v )
	{
		return (Clamp( v ) + 1.0) * 0.5;
	}

	inline std::string Trim( const std::string& s )
	{
		const size_t a = s.find_first_not_of( " \t\r\n" );
		if (a == std::string::npos) return "";
		const size_t b = s.find_last_not_of( " \t\r\n" );
		return s.substr( a, b - a + 1 );
	}

	/**
	 * Columns of a line of a configuration file: separated by tabs or commas, "#" starts a comment.
	 * @return empty if the line is empty or only a comment
	 */
	inline std::vector<std::string> SplitColumns( std::string line )
	{
		std::vector<std::string> col;
		const size_t comment = line.find( '#' );
		if (comment != std::string::npos) line.erase( comment );
		if (Trim( line ).empty()) return col;

		std::stringstream ss( line );
		std::string token;
		while (std::getline( ss, token, '\t' ))
		{
			std::stringstream ss2( token );
			std::string t2;
			while (std::getline( ss2, t2, ',' )) col.push_back( Trim( t2 ) );
		}
		return col;
	}
	/** Names of the inputs in the column INPUT of the files, in the order of HandControllers::Input. */
	inline constexpr const char* const INPUT_NAMES[] = {
		"RHC_PITCH", "RHC_YAW", "RHC_ROLL", "THC_X", "THC_Y", "THC_Z", "RPTA", "SBTC", "BRAKE_LEFT", "BRAKE_RIGHT",
		"RMS_CAPTURE", "RMS_RELEASE", "RMS_RATE_HOLD", "RMS_RATE_TOGGLE", "RMS_CONTROL_TOGGLE"
	};
	inline constexpr int INPUT_NAME_COUNT = sizeof( INPUT_NAMES ) / sizeof( INPUT_NAMES[0] );

	/** Kinds of input of a game controller. */
	enum SubDeviceType {SUB_AXIS = 0, SUB_BUTTON, SUB_POV};

	/** Identifiers of the axes of DIJOYSTATE2. */
	enum {AX_X = 0, AX_Y, AX_Z, AX_RX, AX_RY, AX_RZ, AX_SLIDER0, AX_SLIDER1};

	/** Actions of a button on a switch or pushbutton of a panel (the same values as HandControllers::PanelAction). */
	enum {ACT_UP = 0, ACT_DOWN, ACT_MID, ACT_TOGGLE, ACT_HOLD_UP, ACT_HOLD_DOWN, ACT_PRESS};

	inline std::string Lower( std::string s )
	{
		std::transform( s.begin(), s.end(), s.begin(), []( unsigned char c ) { return (char)std::tolower( c ); } );
		return s;
	}

	/**
	 * Input of a controller in the column SUBDEVID of the files: "Axis X", "Axis RZ", "Slider 0", "Button 5", "POV 0H".
	 * For a hat the id is the number of the hat * 2 (+1 for the vertical direction).
	 */
	inline bool ParseSubDevice( const std::string& text, int& type, int& id )
	{
		const std::string t = Lower( Trim( text ) );
		if (t.compare( 0, 4, "axis" ) == 0)
		{
			const std::string a = Trim( t.substr( 4 ) );
			type = SUB_AXIS;
			if (a == "x") id = AX_X;
			else if (a == "y") id = AX_Y;
			else if (a == "z") id = AX_Z;
			else if (a == "rx") id = AX_RX;
			else if (a == "ry") id = AX_RY;
			else if (a == "rz") id = AX_RZ;
			else return false;
			return true;
		}
		else if (t.compare( 0, 6, "slider" ) == 0)
		{
			const std::string a = Trim( t.substr( 6 ) );
			type = SUB_AXIS;
			if (a == "0") id = AX_SLIDER0;
			else if (a == "1") id = AX_SLIDER1;
			else return false;
			return true;
		}
		else if (t.compare( 0, 6, "button" ) == 0)
		{
			const std::string a = Trim( t.substr( 6 ) );
			if (a.empty() || (a.find_first_not_of( "0123456789" ) != std::string::npos)) return false;
			const int n = atoi( a.c_str() );
			if ((n < 0) || (n > 127)) return false;
			type = SUB_BUTTON;
			id = n;
			return true;
		}
		else if (t.compare( 0, 3, "pov" ) == 0)
		{
			// "POV 0H": horizontal (+1 = right), "POV 0V": vertical (+1 = down)
			const std::string a = Trim( t.substr( 3 ) );
			if (a.length() != 2) return false;
			const int n = a[0] - '0';
			if ((n < 0) || (n > 3)) return false;
			if (a[1] == 'h') id = n * 2;
			else if (a[1] == 'v') id = (n * 2) + 1;
			else return false;
			type = SUB_POV;
			return true;
		}
		return false;
	}

	/** Input of a button on a panel: "PANEL A8A1/BRAKES:UP" (panel, component, action). */
	inline bool ParsePanelInput( const std::string& text, std::string& panel, std::string& component, int& action )
	{
		const size_t sp = text.find( ' ' );
		if (sp == std::string::npos) return false;
		const std::string rest = Trim( text.substr( sp + 1 ) );
		const size_t slash = rest.find( '/' );
		const size_t colon = rest.rfind( ':' );
		if ((slash == std::string::npos) || (colon == std::string::npos) || (colon < slash)) return false;
		panel = Trim( rest.substr( 0, slash ) );
		component = Trim( rest.substr( slash + 1, colon - slash - 1 ) );
		const std::string a = Lower( Trim( rest.substr( colon + 1 ) ) );
		if (a == "up") action = ACT_UP;
		else if (a == "down") action = ACT_DOWN;
		else if (a == "mid") action = ACT_MID;
		else if (a == "toggle") action = ACT_TOGGLE;
		else if (a == "hold_up") action = ACT_HOLD_UP;
		else if (a == "hold_down") action = ACT_HOLD_DOWN;
		else if (a == "press") action = ACT_PRESS;
		else return false;
		return !panel.empty() && !component.empty();
	}

}


#endif// _CONTROLLERMATH_H_
