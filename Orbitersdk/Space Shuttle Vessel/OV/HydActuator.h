/****************************************************************************
  This file is part of Space Shuttle Vessel

  Power of the hydraulic actuators of the aerosurfaces, brakes and nose wheel steering from the pressure of each of the three hydraulic
  systems (no Orbiter calls, so it can be tested alone)

  Data (Crew Operations Manual OI-27, sections 2.1 and 2.6; "Single APU Landing" of the JSC; NASA TM-81366):
  - every elevon has a switching valve with three sources, the primary and two standby ones, that switches when the pressure of the source
    falls to 1200-1500 psia; the sources (primary / standby 1 / standby 2) of the elevons, as in the example of DISP 87, are: left outboard
    3/1/2, left inboard 2/1/3, right inboard 3/1/2, right outboard 2/1/3
  - the elevon rate is limited to 20 deg/s, 13.9 deg/s in flight with a single APU (the four elevons at the full rate use nearly the whole
    flow of one APU)
  - the rudder/speedbrake and the body flap have three hydraulic motors each, one per system; the rudder/speedbrake with two of the three
    motors failed gives about half of the output (5.43 deg/s of the speedbrake to open with a single APU)
  - the brakes use systems 1 and 2 (the regulators drop the pressure to 2000 psig), and system 3 takes over below about 1000 psi; without
    system 3 and with one of the others lost the braking power is half
  - the nose wheel steering uses system 1 and then 2; the actuator stays in caster mode if both are below about 1325 psi
  The data of the elevons (behavior with no pressure) and the share of the power at the pressures between the limits are not published:
  they are the estimates of this file, marked "estimate".

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
#ifndef _HYDACTUATOR_H_
#define _HYDACTUATOR_H_


#include <cmath>


namespace hydact
{
	inline constexpr double P_FULL = 2800.0;// [psi] from here the actuators have the full power (all the systems have more at T-4 minutes)
	inline constexpr double P_NONE = 1000.0;// [psi] estimate: below this no power (the brake takeover of the system 3 is also here)
	inline constexpr double P_SWITCH = 1350.0;// [psi] the switching valve of an elevon leaves a source at 1200-1500 psia
	inline constexpr double P_NWS_CASTER = 1325.0;// [psi] the nose wheel steering is in caster mode with both systems below this

	inline constexpr double ELEVON_RATE = 20.0;// [deg/s]
	inline constexpr double ELEVON_RATE_ONE_SYSTEM = 13.9;// [deg/s] the four elevons on the same system
	inline constexpr double RSB_RATE_ONE_MOTOR = 5.43;// [deg/s] of the speedbrake to open, a single motor (half of the full rate)
	inline constexpr double BODYFLAP_RATE = 2.25;// [deg/s] without hinge moment (1-3 deg/s)

	inline constexpr int ELEVONS = 4;// left outboard, left inboard, right inboard, right outboard
	/** The three sources (systems 0, 1 and 2: primary, standby 1, standby 2) of every elevon. */
	inline constexpr int ELEVON_SOURCE[ELEVONS][3] = {{2, 0, 1}, {1, 0, 2}, {2, 0, 1}, {1, 0, 2}};

	/** Share of the power [0, 1] of the actuators of a system with this pressure (estimate: linear between P_NONE and P_FULL). */
	inline double Power( double pressure )
	{
		if (pressure <= P_NONE) return 0.0;
		if (pressure >= P_FULL) return 1.0;
		return (pressure - P_NONE) / (P_FULL - P_NONE);
	}

	/**
	 * System that feeds an elevon (the switching valve): the first source of the three that has pressure, from the primary one.
	 * @param pressure pressures of the three systems [psi]
	 * @param offset change of the pressure of the switching of the valve of this vehicle [psi]
	 * @return the system (0, 1, 2), or -1 if none has pressure
	 */
	inline int ElevonSystem( const double* pressure, int elevon, double offset = 0.0 )
	{
		for (int s = 0; s < 3; s++)
		{
			if (pressure[ELEVON_SOURCE[elevon][s]] >= (P_SWITCH + offset)) return ELEVON_SOURCE[elevon][s];
		}
		return -1;
	}

	/** Rate limit of an elevon [deg/s] with this number of elevons on its system (2 or less: the full rate; 4: the one of a single system). */
	inline double ElevonRate( int sharing )
	{
		if (sharing <= 2) return ELEVON_RATE;
		if (sharing >= ELEVONS) return ELEVON_RATE_ONE_SYSTEM;
		return ELEVON_RATE - ((double)(sharing - 2) * (ELEVON_RATE - ELEVON_RATE_ONE_SYSTEM) / (double)(ELEVONS - 2));
	}

	/**
	 * Number of elevons on the systems of the pressures, for each elevon (how many are fed by the same system as this one).
	 * @param sharing output [ELEVONS]; 0 for an elevon with no source
	 */
	inline void ElevonSharing( const double* pressure, const double* offset, int* sharing )
	{
		int system[ELEVONS];
		for (int e = 0; e < ELEVONS; e++) system[e] = ElevonSystem( pressure, e, (offset != nullptr) ? offset[e] : 0.0 );
		for (int e = 0; e < ELEVONS; e++)
		{
			sharing[e] = 0;
			if (system[e] < 0) continue;
			for (int o = 0; o < ELEVONS; o++) sharing[e] += (system[o] == system[e]) ? 1 : 0;
		}
	}

	/**
	 * Output of a drive with three motors, one per system (the rudder/speedbrake and the body flap), relative to the full rate [0, 1]:
	 * 1 for the three motors, 0.5 for one (the manual: about half). The output with two motors is an estimate.
	 * @param power the power of each of the three systems [0, 1]
	 */
	inline double MotorDriveOutput( const double* power )
	{
		const double table[4] = {0.0, 0.5, 0.8, 1.0};
		double n = 0.0;
		for (int s = 0; s < 3; s++) n += power[s];// the motors with a power below the full one add less
		if (n <= 0.0) return 0.0;
		const int lo = (n >= 3.0) ? 3 : static_cast<int>(floor( n ));
		const int hi = (lo >= 3) ? 3 : (lo + 1);
		const double f = n - (double)lo;
		const double out = table[lo] + ((table[hi] - table[lo]) * f);
		return (out < 0.0) ? 0.0 : ((out > 1.0) ? 1.0 : out);
	}

	/** Power of the brakes [0, 1] from the pressures (systems 1 and 2, system 3 takes the place of one that is lost, 1000 psi). */
	inline double BrakePower( const double* pressure )
	{
		const bool ok1 = pressure[0] > P_NONE, ok2 = pressure[1] > P_NONE, ok3 = pressure[2] > P_NONE;
		double a = ok1 ? Power( pressure[0] ) : 0.0;
		double b = ok2 ? Power( pressure[1] ) : 0.0;
		if (ok3)
		{
			// the system 3 takes the place of one of the systems that are lost (only one)
			if (!ok1) a = Power( pressure[2] );
			else if (!ok2) b = Power( pressure[2] );
		}
		return 0.5 * (a + b);
	}

	/** The nose wheel steering has power (system 1, then 2): false = caster mode. */
	inline bool SteeringPower( const double* pressure )
	{
		return (pressure[0] >= P_NWS_CASTER) || (pressure[1] >= P_NWS_CASTER);
	}

	// dispersions of this file (1 sigma): the pressure of the switching of the valve of an elevon is in 1200-1500 psi (3 sigma), the rest are estimates
	inline constexpr double SWITCH_SIGMA = 50.0;// [psi]
	inline constexpr double RATE_SIGMA = 0.03;// relative; estimate
	inline constexpr double RATE_NOISE_SIGMA = 0.01;// relative, at every use; estimate
	inline constexpr double BRAKE_GAIN_SIGMA = 0.04;// relative; estimate
	inline constexpr double PUMP_SIGMA = 33.0;// [psi] the regulation of the pump in NORM is 2900-3100 psi (3 sigma)
	inline constexpr double PUMP_LOW_SIGMA = 80.0;// [psi] in LOW it is 500-1000 psi, about 800 at the start (3 sigma = 250)
	inline constexpr double PUMP_NOISE_SIGMA = 15.0;// [psi] slow variation of the set point at each second; estimate

	/** Moves a position toward the command at most max step (the rate limit times the time). */
	inline double Slew( double position, double command, double maxStep )
	{
		const double d = command - position;
		if (d > maxStep) return position + maxStep;
		if (d < -maxStep) return position - maxStep;
		return command;
	}

	// elevons with no pressure (estimate): the elevon floats with the flow, and the loads that the actuators can not hold push it
	inline constexpr double FLOAT_PER_ALPHA = 0.5;// [deg/deg] the trailing edge goes up with the angle of attack (the flow turns it)
	inline constexpr double FLOAT_MIN = -35.0, FLOAT_MAX = 20.0;// [deg]
	inline constexpr double FLOAT_TAU_AT_Q = 3.0;// [s] time constant of the float at the dynamic pressure of FLOAT_Q
	inline constexpr double FLOAT_Q = 1000.0;// [Pa]
	inline constexpr double HINGE_Q = 12000.0;// [Pa] estimate: dynamic pressure at which the aerodynamic load is the full capacity of an actuator

	/** Angle where the free elevon settles [deg] from the angle of attack [deg]. */
	inline double ElevonFloatAngle( double alphaDeg )
	{
		const double a = -FLOAT_PER_ALPHA * alphaDeg;
		return (a < FLOAT_MIN) ? FLOAT_MIN : ((a > FLOAT_MAX) ? FLOAT_MAX : a);
	}

	/** Step of an elevon that floats (no power): exponential approach to the free angle, faster with the dynamic pressure. */
	inline double FloatStep( double position, double floatAngle, double dynPressure, double dt )
	{
		// with no air flow (on the ground, in orbit) nothing moves the surface: the fluid in the actuators holds it
		const double fade = (dynPressure - 20.0) / 180.0;
		if (fade <= 0.0) return position;
		const double w = (fade > 1.0) ? 1.0 : fade;
		const double q = (dynPressure < 50.0) ? 50.0 : dynPressure;
		double tau = FLOAT_TAU_AT_Q * (FLOAT_Q / q);
		if (tau < 0.3) tau = 0.3;
		if (tau > 120.0) tau = 120.0;
		return position + ((floatAngle - position) * (1.0 - exp( -dt / tau )) * w);
	}

	/**
	 * Share [0, 1] of the aerodynamic load that the actuator can not hold at the power and the dynamic pressure (0 = holds it all); the load
	 * is estimated from the dynamic pressure and the distance of the elevon to its free angle, the capacity from the power.
	 */
	inline double BlowbackShare( double power, double dynPressure, double position, double floatAngle )
	{
		const double load = (dynPressure / HINGE_Q) * (fabs( position - floatAngle ) / 30.0);
		if (load <= power) return 0.0;
		const double excess = (load - power) / load;
		return (excess < 0.0) ? 0.0 : ((excess > 1.0) ? 1.0 : excess);
	}
}


#endif// _HYDACTUATOR_H_
