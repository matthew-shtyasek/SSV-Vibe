/****************************************************************************
  This file is part of Space Shuttle Vessel

  Model of the shutdown, the cooldown and the restart of the APUs (no Orbiter calls, so it can be tested alone)

  The numbers are from the Space Shuttle Crew Operations Manual (USA007587, section 2.1 APU/HYD):
  - auto shutdown below 80% (57600 rpm) and above 129% (92880 rpm) of the speed, the underspeed check starts 10.5 s after the start command,
    the overspeed check has no delay; the shutdown closes the secondary fuel control valve and the fuel tank isolation valves
  - normal speed 103%, HIGH speed 113%
  - restart limits: injector branch temperature 400 F, fuel pump 210 F, gas generator valve module 200 F (above them the hydrazine can
    detonate); the passive cooldown takes about 180 minutes, the injector water cooling needs 3.5 minutes of continuous flow
  - one water tank for the three APUs: 21 minutes of continuous flow (six cycles of 3.5 minutes), 9 lb of water

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
#ifndef _APUMODEL_H_
#define _APUMODEL_H_


#include <algorithm>
#include <cmath>


namespace apumodel
{
	// speed [% of the nominal speed]
	inline constexpr double UNDERSPEED_PCT = 80.0;
	inline constexpr double OVERSPEED_PCT = 129.0;
	inline constexpr double STARTUP_DELAY = 10.5;// [s] the underspeed check is inhibited after the start command
	inline constexpr double SPEED_NORM_PCT = 103.0;
	inline constexpr double SPEED_HIGH_PCT = 113.0;

	// restart limits [F]
	inline constexpr double LIMIT_INJECTOR_F = 400.0;
	inline constexpr double LIMIT_PUMP_F = 210.0;
	inline constexpr double LIMIT_VALVE_F = 200.0;

	// thermal model [F, s]
	inline constexpr double AMBIENT_F = 70.0;
	inline constexpr double PASSIVE_TAU = 190.0 * 60.0;// the temperatures are at the limits about 180 minutes after the shutdown
	inline constexpr double PEAK_INJECTOR_F = 920.0;// soakback right after the shutdown
	inline constexpr double PEAK_PUMP_F = 195.0;// passive cooling: inside the limit after a normal shutdown
	inline constexpr double PEAK_VALVE_F = 185.0;
	inline constexpr double RUN_INJECTOR_F = 300.0;
	inline constexpr double RUN_PUMP_F = 170.0;
	inline constexpr double RUN_VALVE_F = 150.0;
	inline constexpr double COOLING_INJECTOR_F = 250.0;// the injector with the water flow tends to this temperature
	inline constexpr double TAU_RUN = 120.0;
	inline constexpr double TAU_SOAK = 180.0;
	inline constexpr double TAU_INJECTOR_COOLING = 143.0;// from the soakback peak to 400 F in 3.5 minutes
	inline constexpr double TAU_INJECTOR_REHEAT = 20.0;// the injector gets hot again quickly when the cooling stops

	// injector cooling water
	inline constexpr double WATER_FLOW_TIME = 21.0 * 60.0;// [s] of flow of one valve that the tank supplies
	inline constexpr double COOLING_CYCLE = 3.5 * 60.0;// [s]

	struct Thermal
	{
		double injector = AMBIENT_F;// injector branch temperature [F]
		double pump = AMBIENT_F;// fuel pump temperature [F]
		double valve = AMBIENT_F;// gas generator valve module temperature [F]
		double soak = 0.0;// remaining heat of the last run: 1 right after a run, decays with PASSIVE_TAU
		bool reheat = false;// the injector was cooled with water and is heating up again
	};

	/** Temperatures of an APU that is cold for a long time, or that is running. */
	inline Thermal ColdThermal( void )
	{
		return Thermal();
	}

	inline Thermal RunningThermal( void )
	{
		Thermal t;
		t.injector = RUN_INJECTOR_F;
		t.pump = RUN_PUMP_F;
		t.valve = RUN_VALVE_F;
		t.soak = 1.0;
		return t;
	}

	/** First order lag of a value to a target. */
	inline double Relax( double value, double target, double tau, double dt )
	{
		if (tau <= 0.0) return target;
		return target + ((value - target) * exp( -dt / tau ));
	}

	/**
	 * Advances the temperatures of an APU.
	 * @param running the APU is running (or starting)
	 * @param injectorWater the injector water cooling is on and there is water
	 */
	inline void ThermalStep( Thermal& t, bool running, bool injectorWater, double dt )
	{
		if (dt <= 0.0) return;

		if (running)
		{
			t.soak = 1.0;
			t.reheat = false;
			t.injector = Relax( t.injector, RUN_INJECTOR_F, TAU_RUN, dt );
			t.pump = Relax( t.pump, RUN_PUMP_F, TAU_RUN, dt );
			t.valve = Relax( t.valve, RUN_VALVE_F, TAU_RUN, dt );
			return;
		}

		t.soak *= exp( -dt / PASSIVE_TAU );
		const double tpump = AMBIENT_F + ((PEAK_PUMP_F - AMBIENT_F) * t.soak);
		const double tvalve = AMBIENT_F + ((PEAK_VALVE_F - AMBIENT_F) * t.soak);
		const double tinj = AMBIENT_F + ((PEAK_INJECTOR_F - AMBIENT_F) * t.soak);
		t.pump = Relax( t.pump, tpump, TAU_SOAK, dt );
		t.valve = Relax( t.valve, tvalve, TAU_SOAK, dt );

		if (injectorWater)
		{
			t.injector = Relax( t.injector, (std::min)( COOLING_INJECTOR_F, tinj ), TAU_INJECTOR_COOLING, dt );
			t.reheat = true;
		}
		else
		{
			// the soakback is slow, the reheat of a cooled injector is quick
			const double tau = (t.reheat && (t.injector < tinj)) ? TAU_INJECTOR_REHEAT : TAU_SOAK;
			t.injector = Relax( t.injector, tinj, tau, dt );
			if (fabs( t.injector - tinj ) < 1.0) t.reheat = false;
		}
		return;
	}

	/** True if the temperatures are inside the limits of a safe start. */
	inline bool SafeToStart( const Thermal& t )
	{
		return (t.injector <= LIMIT_INJECTOR_F) && (t.pump <= LIMIT_PUMP_F) && (t.valve <= LIMIT_VALVE_F);
	}

	/**
	 * Probability that the hydrazine detonates when an APU is started (or the fuel reaches the gas generator) with these temperatures:
	 * zero inside the limits, one when they are exceeded by 200 F (injector) or by 60 F (pump, valve module).
	 */
	inline double DetonationProbability( const Thermal& t )
	{
		double p = 0.0;
		if (t.injector > LIMIT_INJECTOR_F) p = (std::max)( p, (std::min)( 1.0, (t.injector - LIMIT_INJECTOR_F) / 200.0 ) );
		if (t.pump > LIMIT_PUMP_F) p = (std::max)( p, (std::min)( 1.0, (t.pump - LIMIT_PUMP_F) / 60.0 ) );
		if (t.valve > LIMIT_VALVE_F) p = (std::max)( p, (std::min)( 1.0, (t.valve - LIMIT_VALVE_F) / 60.0 ) );
		return p;
	}

	/** Speed that the controller holds [%]. */
	inline double TargetSpeed( bool high )
	{
		return high ? SPEED_HIGH_PCT : SPEED_NORM_PCT;
	}

	/**
	 * The underspeed condition: below 80% of the speed, only after the start delay.
	 * @param timeSinceStart time since the start command [s]
	 */
	inline bool UnderspeedTrip( double speedPct, double timeSinceStart )
	{
		return (timeSinceStart >= STARTUP_DELAY) && (speedPct < UNDERSPEED_PCT);
	}

	/** The overspeed condition: above 129% of the speed, no delay. */
	inline bool OverspeedTrip( double speedPct )
	{
		return speedPct > OVERSPEED_PCT;
	}

	/** Water of the injector cooling tank that is left [s of flow of one valve] after dt with this number of valves open. */
	inline double WaterAfter( double water, int valvesOpen, double dt )
	{
		return (std::max)( 0.0, water - (valvesOpen * dt) );
	}
}


#endif// _APUMODEL_H_
