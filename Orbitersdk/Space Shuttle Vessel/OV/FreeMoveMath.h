/****************************************************************************
  This file is part of Space Shuttle Vessel

  Free movement of the head in the virtual cockpit: the position of the camera moves around the position of the station with the keys,
  limited to a small ellipsoid (no Orbiter calls, so it can be tested alone)

  The frame is the one of the cockpit: x right, y up, z forward.


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
#ifndef _FREEMOVEMATH_H_
#define _FREEMOVEMATH_H_


#include <OrbiterAPI.h>
#include <cmath>


namespace freemove
{
	inline constexpr double SPEED = 0.35;// [m/s] speed of the head
	inline constexpr double RADIUS_HORIZONTAL = 0.5;// [m] distance from the position of the station on the horizontal plane
	inline constexpr double RADIUS_UP = 0.3;// [m] up
	inline constexpr double RADIUS_DOWN = 0.4;// [m] down
	inline constexpr double MAX_STEP = 0.1;// [s] longest time step used

	/**
	 * Limits an offset from the station to the ellipsoid of the allowed positions (the part up and the part down have different heights).
	 * A point inside is not changed; outside, it is moved along the line to the center to the surface.
	 */
	inline VECTOR3 ClampOffset( const VECTOR3& o )
	{
		const double ry = (o.y >= 0.0) ? RADIUS_UP : RADIUS_DOWN;
		const double qx = o.x / RADIUS_HORIZONTAL;
		const double qz = o.z / RADIUS_HORIZONTAL;
		const double qy = o.y / ry;
		const double n = sqrt( (qx * qx) + (qy * qy) + (qz * qz) );
		if (n <= 1.0) return o;
		return _V( o.x / n, o.y / n, o.z / n );
	}

	/** @return true if the offset is inside the ellipsoid (small tolerance for the numbers) */
	inline bool IsInside( const VECTOR3& o )
	{
		const double ry = (o.y >= 0.0) ? RADIUS_UP : RADIUS_DOWN;
		const double qx = o.x / RADIUS_HORIZONTAL;
		const double qz = o.z / RADIUS_HORIZONTAL;
		const double qy = o.y / ry;
		return ((qx * qx) + (qy * qy) + (qz * qz)) <= 1.0000001;
	}

	/**
	 * Forward direction on the horizontal plane from the direction of the view.
	 * @param view direction of the view (not necessarily a unit vector)
	 * @param fallback used when the view is (almost) vertical
	 */
	inline VECTOR3 HorizontalForward( const VECTOR3& view, const VECTOR3& fallback )
	{
		double h = sqrt( (view.x * view.x) + (view.z * view.z) );
		if (h > 0.05) return _V( view.x / h, 0.0, view.z / h );
		h = sqrt( (fallback.x * fallback.x) + (fallback.z * fallback.z) );
		if (h > 1e-9) return _V( fallback.x / h, 0.0, fallback.z / h );
		return _V( 0.0, 0.0, 1.0 );
	}

	/** Direction to the right of a horizontal forward direction (x right, y up, z forward). */
	inline VECTOR3 RightOf( const VECTOR3& forward )
	{
		return _V( forward.z, 0.0, -forward.x );
	}

	/**
	 * Moves the head.
	 * @param offset offset from the station [m]
	 * @param fwd input forward (+1) or back (-1)
	 * @param right input right (+1) or left (-1)
	 * @param up input up (+1) or down (-1)
	 * @param forward horizontal forward direction of the view (unit vector, see HorizontalForward)
	 * @param dt time step [s]
	 * @return the new offset, inside the ellipsoid
	 */
	inline VECTOR3 Move( const VECTOR3& offset, double fwd, double right, double up, const VECTOR3& forward, double dt )
	{
		if (dt > MAX_STEP) dt = MAX_STEP;
		if (dt <= 0.0) return offset;
		const VECTOR3 r = RightOf( forward );
		VECTOR3 dir = (forward * fwd) + (r * right) + _V( 0.0, up, 0.0 );
		const double l = sqrt( (dir.x * dir.x) + (dir.y * dir.y) + (dir.z * dir.z) );
		if (l < 1e-9) return ClampOffset( offset );// no input (a pair of opposite keys): at most, the clamp of a position out of the limits
		if (l > 1.0) dir = dir / l;// the diagonals are not faster
		return ClampOffset( offset + (dir * (SPEED * dt)) );
	}

	/** Key (with Ctrl) of the choice of the setting: the order of settings::FREE_MOVE_KEYS. Zero if the choice does not exist. */
	inline unsigned int ToggleKey( int choice )
	{
		switch (choice)
		{
			case 0: return OAPI_KEY_HOME;
			case 1: return OAPI_KEY_END;
			case 2: return OAPI_KEY_INSERT;
			case 3: return OAPI_KEY_DELETE;
			case 4: return OAPI_KEY_SCROLL;
			case 5: return OAPI_KEY_GRAVE;
		}
		return 0;
	}

	/** Stations in which the head can be moved (the cameras and the airlock have a fixed position). */
	inline bool StationAllowsMove( int vcMode, int firstCamera, int lastCamera, int airlockFirst )
	{
		if ((vcMode >= firstCamera) && (vcMode <= lastCamera)) return false;
		if (vcMode >= airlockFirst) return false;
		return true;
	}
}


#endif// _FREEMOVEMATH_H_
