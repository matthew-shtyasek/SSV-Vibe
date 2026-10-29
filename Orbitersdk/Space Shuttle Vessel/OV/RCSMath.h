/****************************************************************************
  This file is part of Space Shuttle Vessel

  Thrust of the RCS commanded by the DAPs with time acceleration (no Orbiter calls, so it can be tested alone)

  Each step of the simulation is as long as the time acceleration times the real time of the frame, and the jets fire the whole step. To keep
  the control of the attitude (a jet that fires for a long step gives a large change of the rate), the DAPs reduce the thrust that they command
  by the time acceleration. Up to a time acceleration the thrust is not reduced (the maneuvers take the same time as at no acceleration and the
  flames are seen); above it the reduction is the acceleration divided by this one.


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
#ifndef _RCSMATH_H_
#define _RCSMATH_H_


namespace rcsmath
{
	/** Shortest firing of a jet [s] (the minimum on-time of the valve): in real time the rates of the DAP cannot be finer than the impulse of such a firing. */
	inline constexpr double MIN_PULSE = 0.08;

	/**
	 * Factor (1 or more) by which the DAPs divide the thrust of the RCS and the torque that they use for their limits: the time acceleration. The
	 * steps of the simulation are as many times longer, and the impulse of a step is that of a step in real time, so the hold, the rates and the
	 * propellant per correction do not depend on the acceleration; the torque left (the torque divided by this factor) limits the acceleration with
	 * which the DAP can hold the attitude (the orbit DAP holds on rails above it, see OnRails).
	 * Tried and dropped: the thrust sized by the step to the shortest firing (0.08 s) made the primary jets worse than the original law at x10 and x100.
	 */
	inline double ThrustReduction( double acceleration )
	{
		return (acceleration > 1.0) ? acceleration : 1.0;
	}

	/** The DAP holds the attitude on rails (without firing the jets) from this time acceleration (0 or 1: never). */
	inline bool OnRails( double acceleration, double railsAbove )
	{
		return (railsAbove > 1.0) && (acceleration >= railsAbove);
	}

	/**
	 * Mean propellant use of the attitude hold with the vernier jets [kg/hour] (JSC planning 1977, Table II, 100 nmi: 0.7 lb/hr in LVLH, 3.6 lb/hr
	 * inertial in the Y-POP attitude); the DAP on rails subtracts it from the tanks.
	 */
	inline double RailsPropellantRate( bool lvlh )
	{
		return lvlh ? 0.32 : 1.6;
	}
}


#endif// _RCSMATH_H_
