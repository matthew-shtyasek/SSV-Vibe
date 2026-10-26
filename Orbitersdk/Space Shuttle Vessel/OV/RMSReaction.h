/****************************************************************************
  This file is part of Space Shuttle Vessel

  Reaction of the RMS on the orbiter: the payload that the arm moves accelerates relative to the orbiter, and the orbiter feels the
  opposite force at the payload (no Orbiter calls, so it can be tested alone)

  The arm is a massless link in this model: the force that the arm applies on the payload at its center of mass is applied with the
  opposite sign on the orbiter, at the same point (the lever arm of the force is the payload position in the orbiter frame, so it also
  gives the torque on the orbiter).


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
#ifndef _RMSREACTION_H_
#define _RMSREACTION_H_


#include <OrbiterAPI.h>
#include <cmath>


namespace rmsreact
{
	inline constexpr double FILTER_TAU = 0.5;// [s] time constant of the filter of the velocity
	inline constexpr double MAX_ACCELERATION = 0.2;// [m/s^2] limit of the estimated acceleration (the arm does not do more, a jump of the position does)

	/**
	 * Estimator of the acceleration of a point from its positions at every step: the velocity is made with the difference of the positions and
	 * filtered, and the acceleration is its derivative, so the noise of the positions does not become force.
	 */
	class AccelEstimator
	{
	public:
		AccelEstimator( void )
		{
			Reset();
		}

		void Reset( void )
		{
			count = 0;
			lastPos = _V( 0.0, 0.0, 0.0 );
			vf = _V( 0.0, 0.0, 0.0 );
			lastVf = _V( 0.0, 0.0, 0.0 );
		}

		/**
		 * @param pos position [m]
		 * @param dt time step [s]
		 * @param[out] acc acceleration [m/s^2]
		 * @return false while the estimator has no data for an acceleration (the first two steps)
		 */
		bool Update( const VECTOR3& pos, double dt, VECTOR3& acc )
		{
			acc = _V( 0.0, 0.0, 0.0 );
			if (dt <= 0.0) return false;

			if (count == 0)
			{
				lastPos = pos;
				count = 1;
				return false;
			}
			const VECTOR3 v = (pos - lastPos) / dt;
			lastPos = pos;
			if (count == 1)
			{
				vf = v;
				lastVf = vf;
				count = 2;
				return false;
			}

			const double alpha = 1.0 - exp( -dt / FILTER_TAU );
			vf = vf + ((v - vf) * alpha);
			acc = (vf - lastVf) / dt;
			lastVf = vf;

			const double a = length( acc );
			if (a > MAX_ACCELERATION) acc = acc * (MAX_ACCELERATION / a);
			return true;
		}

	private:
		int count;
		VECTOR3 lastPos;
		VECTOR3 vf;
		VECTOR3 lastVf;
	};

	/**
	 * Force on the orbiter from the acceleration of the payload relative to it.
	 * @param mass mass of the payload [kg]
	 * @param acc acceleration of the payload relative to the orbiter [m/s^2]
	 */
	inline VECTOR3 ReactionForce( double mass, const VECTOR3& acc )
	{
		return acc * (-mass);
	}

	/** Torque on the orbiter about its center of mass of a force applied at the position r (same frame). */
	inline VECTOR3 ReactionTorque( const VECTOR3& r, const VECTOR3& force )
	{
		return crossp( r, force );
	}
}


#endif// _RMSREACTION_H_
