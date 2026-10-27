/****************************************************************************
  This file is part of Space Shuttle Vessel

  Attitude angles of the ADI for the three positions of the ADI ATTITUDE switch (no Orbiter calls, so it can be tested alone)

  The attitude is a direction cosine matrix D (M50 to body: a vector in the body axes is D times the vector in M50, the rows of D are the
  body axes in M50). The ADI shows the angles (roll, pitch, yaw: sequence pitch, yaw, roll, as the UNIV PTG display) of the body
  relative to a frame F, with the matrix of F to the body = D * (D of F) transposed:
    INRTL: the inertial frame (the angles of the UNIV PTG display)
    LVLH:  the frame of the orbiter in orbit: X along the velocity (horizontal), Z toward the center of the Earth, Y = Z x X
    REF:   the body axes at the moment when the ATT REF pushbutton was last pressed (0, 0, 0 then)


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
#ifndef _ADIMATH_H_
#define _ADIMATH_H_


#include <cmath>


namespace adimath
{
	/** Positions of the switch ADI ATTITUDE (the index of its labels). */
	enum Reference {REF_REF = 0, REF_LVLH = 1, REF_INRTL = 2};

	struct Mat3
	{
		double m[3][3];
	};

	inline Mat3 Identity( void )
	{
		Mat3 r = {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
		return r;
	}

	inline Mat3 Transpose( const Mat3& a )
	{
		Mat3 r;
		for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) r.m[i][j] = a.m[j][i];
		return r;
	}

	inline Mat3 Mul( const Mat3& a, const Mat3& b )
	{
		Mat3 r;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++) r.m[i][j] = (a.m[i][0] * b.m[0][j]) + (a.m[i][1] * b.m[1][j]) + (a.m[i][2] * b.m[2][j]);
		}
		return r;
	}

	/** Matrix (the transformation of the vectors) of a unit quaternion, scalar first, as the quaternion of the attitude processor (M50 to body). */
	inline Mat3 FromQuaternion( double s, double x, double y, double z )
	{
		const double n = std::sqrt( (s * s) + (x * x) + (y * y) + (z * z) );
		if (n > 0.0)
		{
			s /= n;
			x /= n;
			y /= n;
			z /= n;
		}
		else
		{
			s = 1.0;
			x = y = z = 0.0;
		}
		Mat3 r = {{
			{1.0 - (2.0 * ((y * y) + (z * z))), 2.0 * ((x * y) - (s * z)), 2.0 * ((x * z) + (s * y))},
			{2.0 * ((x * y) + (s * z)), 1.0 - (2.0 * ((x * x) + (z * z))), 2.0 * ((y * z) - (s * x))},
			{2.0 * ((x * z) - (s * y)), 2.0 * ((y * z) + (s * x)), 1.0 - (2.0 * ((x * x) + (y * y)))}
		}};
		return r;
	}

	/**
	 * Matrix of the inertial frame to the frame LVLH of the orbiter: the rows are the axes of the frame in the inertial one.
	 * X along the velocity (horizontal), Z toward the center of the Earth, Y = Z x X (against the angular momentum).
	 * @return false if the position and the velocity are parallel or zero
	 */
	inline bool LVLHFrame( const double r[3], const double v[3], Mat3& d )
	{
		const double lr = std::sqrt( (r[0] * r[0]) + (r[1] * r[1]) + (r[2] * r[2]) );
		const double hx = (r[1] * v[2]) - (r[2] * v[1]);
		const double hy = (r[2] * v[0]) - (r[0] * v[2]);
		const double hz = (r[0] * v[1]) - (r[1] * v[0]);
		const double lh = std::sqrt( (hx * hx) + (hy * hy) + (hz * hz) );
		if ((lr < 1.0) || (lh < 1e-6)) return false;
		const double ez[3] = {-r[0] / lr, -r[1] / lr, -r[2] / lr};
		const double ey[3] = {-hx / lh, -hy / lh, -hz / lh};
		const double ex[3] = {(ey[1] * ez[2]) - (ey[2] * ez[1]), (ey[2] * ez[0]) - (ey[0] * ez[2]), (ey[0] * ez[1]) - (ey[1] * ez[0])};
		for (int k = 0; k < 3; k++)
		{
			d.m[0][k] = ex[k];
			d.m[1][k] = ey[k];
			d.m[2][k] = ez[k];
		}
		return true;
	}

	/** Matrix of the frame F to the body, from the matrix of the inertial frame to the body and the one of the inertial frame to F. */
	inline Mat3 Relative( const Mat3& body, const Mat3& frame )
	{
		return Mul( body, Transpose( frame ) );
	}

	/**
	 * Euler angles of the sequence pitch, yaw, roll of the matrix (the angles of the UNIV PTG display and of the ADI), degrees from 0 to 360:
	 * roll positive to the right, pitch positive with the nose up, yaw positive with the nose to the right. The yaw is in -90 to 90 degrees.
	 */
	inline void Angles( const Mat3& d, double& roll, double& pitch, double& yaw )
	{
		const double PI = 3.14159265358979323846;
		const double syaw = d.m[0][1];
		const double cyaw = std::sqrt( std::fmax( 0.0, 1.0 - (syaw * syaw) ) );
		double sroll, croll, spitch, cpitch;
		if (cyaw > 0.005)
		{
			sroll = -d.m[2][1];
			croll = d.m[1][1];
			spitch = -d.m[0][2];
			cpitch = d.m[0][0];
		}
		else
		{
			// the yaw is 90 degrees: the roll is taken as 0
			sroll = 0.0;
			croll = 1.0;
			spitch = d.m[2][0];
			cpitch = d.m[2][2];
		}
		double p = std::atan2( -spitch, -cpitch ) + PI;
		double y = std::atan2( -syaw, -cyaw ) + PI;
		double r = std::atan2( -sroll, -croll ) + PI;
		const double lim = 6.283098;// 359.995 degrees: as in the display, the angles are never 360
		if (p >= lim) p = 0.0;
		if (y >= lim) y = 0.0;
		if (r >= lim) r = 0.0;
		roll = r * 180.0 / PI;
		pitch = p * 180.0 / PI;
		yaw = y * 180.0 / PI;
	}
}


#endif// _ADIMATH_H_
