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
#include <cmath>
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
}


#endif// _CONTROLLERMATH_H_
