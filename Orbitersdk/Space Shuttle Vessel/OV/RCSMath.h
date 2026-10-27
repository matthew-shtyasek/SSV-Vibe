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
	/**
	 * Factor (1 or more) by which the DAPs divide the thrust of the RCS and the torque that they use for their limits.
	 * @param acceleration time acceleration of the simulation
	 * @param fullThrustUpTo time acceleration up to which the thrust is not reduced (1 or less: it is reduced from the first one)
	 */
	inline double ThrustReduction( double acceleration, double fullThrustUpTo )
	{
		const double a = (acceleration > 1.0) ? acceleration : 1.0;
		const double f = (fullThrustUpTo > 1.0) ? fullThrustUpTo : 1.0;
		return (a > f) ? (a / f) : 1.0;
	}
}


#endif// _RCSMATH_H_
