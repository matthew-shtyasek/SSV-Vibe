/****************************************************************************
  This file is part of Space Shuttle Vessel

  Pointing (UNIV PTG tracking) math, without any Orbiter simulation calls so it can be tested alone


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
#ifndef _PTGMATH_H_
#define _PTGMATH_H_


#include <OrbiterAPI.h>
#include <cmath>


namespace ptg
{
	/**
	 * Swaps the Y and Z components (Orbiter left-handed frames to right-handed frames and back).
	 */
	inline VECTOR3 SwapYZ( const VECTOR3& v )
	{
		return _V( v.x, v.z, v.y );
	}

	/**
	 * Unit vector of a right-handed celestial direction.
	 * @param raDeg right ascension [deg]
	 * @param decDeg declination [deg]
	 */
	inline VECTOR3 DirectionFromRADec( double raDeg, double decDeg )
	{
		const double ra = raDeg * RAD;
		const double dec = decDeg * RAD;
		return _V( cos( dec ) * cos( ra ), cos( dec ) * sin( ra ), sin( dec ) );
	}

	/**
	 * Direction of a point in the DAP (M50) frame from a direction in the Orbiter global frame.
	 * @param g direction in the Orbiter global frame (left-handed)
	 * @param J2000toM50 matrix from J2000 to M50
	 */
	inline VECTOR3 GlobalToDAPFrame( const VECTOR3& g, const MATRIX3& J2000toM50 )
	{
		return mul( J2000toM50, SwapYZ( g ) );
	}

	/**
	 * Direction of a celestial point (RA, DEC in M50) in the DAP frame, which is made from the Orbiter global frame (ecliptic), so the obliquity of the Earth is applied.
	 * @param uM50 direction in M50 (equatorial)
	 * @param J2000toM50 matrix from J2000 to M50
	 * @param obliquity Orbiter planet obliquity matrix (equatorial to ecliptic, as returned by oapiGetPlanetObliquityMatrix)
	 */
	inline VECTOR3 CelestialToDAPFrame( const VECTOR3& uM50, const MATRIX3& J2000toM50, const MATRIX3& obliquity )
	{
		const VECTOR3 eq = tmul( J2000toM50, uM50 );// equatorial J2000, right-handed
		const VECTOR3 global = mul( obliquity, SwapYZ( eq ) );// Orbiter global frame
		return GlobalToDAPFrame( global, J2000toM50 );
	}

	/**
	 * Right-handed frame with the Z axis along u, and the Y axis as near to yref as possible (the columns of the matrix are the axes of the frame).
	 * For u = Z axis and yref = Y axis of a frame, the result is that frame.
	 */
	inline MATRIX3 PointingFrame( const VECTOR3& u, const VECTOR3& yref )
	{
		const VECTOR3 z = u / length( u );
		VECTOR3 y = yref - (z * dotp( yref, z ));
		if (length( y ) < 1e-6)
		{
			// yref is along u: use the axis that is the most perpendicular to u
			const double ax = fabs( z.x );
			const double ay = fabs( z.y );
			const double az = fabs( z.z );
			const VECTOR3 alt = ((ax <= ay) && (ax <= az)) ? _V( 1.0, 0.0, 0.0 ) : (((ay <= az) ? _V( 0.0, 1.0, 0.0 ) : _V( 0.0, 0.0, 1.0 )));
			y = alt - (z * dotp( alt, z ));
		}
		y = y / length( y );
		const VECTOR3 x = crossp( y, z );
		return _M( x.x, y.x, z.x,
				   x.y, y.y, z.y,
				   x.z, y.z, z.z );
	}

	/**
	 * Attitude (body to reference frame) that points the body vector at u.
	 * @param u direction to point at, in the reference frame
	 * @param yref reference for the roll angle (the direction that the Y axis of the pointing frame is nearest to)
	 * @param bodyToFrame attitude with respect to the pointing frame (body vector along the Z axis, roll about it), as made by ConvertPYOMToLVLH
	 */
	inline MATRIX3 PointingAttitude( const VECTOR3& u, const VECTOR3& yref, const MATRIX3& bodyToFrame )
	{
		return mul( PointingFrame( u, yref ), bodyToFrame );
	}
}


#endif// _PTGMATH_H_
