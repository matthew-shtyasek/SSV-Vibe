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

	// detonation of the hydrazine and its consequences (estimates of this file: there are no published data)
	inline constexpr double DETONATION_MAX = 0.95;// the probability of a start above the limits never reaches 1
	inline constexpr double DETONATION_SCALE_INJECTOR_F = 150.0;// [F] above the limit at which the probability is 63%
	inline constexpr double DETONATION_SCALE_PUMP_F = 45.0;// the same for the pump and the valve module (the margin of the limits is narrower: 60 F instead of 200 F)
	inline constexpr double LEAK_RATE = 1.0;// [lb/s] hydrazine that leaks from a destroyed APU with the FUEL TK VLV switch in OPEN: a ruptured line of a tank under pressure (estimate: 1/4 inch line, 365 psi, a restricted flow; about 3 lb/s if the line were open)
	inline constexpr double LEAK_SIGMA = 0.3;// relative scatter of the rate of a leak (each failure)
	inline constexpr double LEAK_JITTER = 0.1;// relative variation of the rate (each second)
	inline constexpr double COLLATERAL_EXPLOSION = 0.5;// probability that the explosion of an APU destroys another one too
	inline constexpr double COLLATERAL_FIRE = 0.15;// probability that the fire of a leak that lasts FIRE_SPREAD_TIME destroys another APU
	inline constexpr double FIRE_SPREAD_TIME = 60.0;// [s] of leak
	inline constexpr double HYD_LOSS_EXPLOSION = 0.5;// probability that the explosion breaks the pump or a line of the hydraulic system of the APU: the reservoir leaks (estimate, no source)
	inline constexpr double HYD_LOSS_FIRE = 0.2;// the same for a fire: the hydraulic fluid (MIL-H-83282) burns slowly, the lines burn through (estimate, no source)

	// fire of the hydrazine that leaks: it needs air (on orbit the compartment is purged with nitrogen); the figures are estimates, no source
	inline constexpr double FIRE_MIN_PRESSURE = 10000.0;// [Pa] ambient pressure from which a leak can burn (about 16 km of altitude)
	inline constexpr double IGNITION_RATE = 1.0 / 90.0;// [1/s] ignition of a leak in the air (a mean of 90 s)
	inline constexpr double FIRE_AFTER_LEAK = 45.0;// [s] that a fire lasts after the leak ended
	inline constexpr double EXPLOSION_IGNITES = 0.5;// probability that an explosion in the air starts a fire at once
	inline constexpr double BOOM_TIME = 1.5;// [s] of the flash of an explosion
	inline constexpr double DEBRIS_TIME = 20.0;// [s] of the smoke and debris of an explosion

	/** True if there is air for a fire at this ambient pressure [Pa]. */
	inline bool AirForFire( double pressure )
	{
		return pressure >= FIRE_MIN_PRESSURE;
	}

	/** Probability that a leak ignites in a step dt [s] at the ambient pressure [Pa]. */
	inline double IgnitionProbability( double dt, double pressure )
	{
		if (!AirForFire( pressure ) || (dt <= 0.0)) return 0.0;
		return 1.0 - std::exp( -IGNITION_RATE * dt );
	}

	/** Probability that a working APU starts to leak and burn at random (the STS-9 kind) in a step dt [s], with the rate [1/hour] of the option. */
	inline double RandomFireProbability( double dt, double ratePerHour, double pressure )
	{
		if (!AirForFire( pressure ) || (dt <= 0.0) || (ratePerHour <= 0.0)) return 0.0;
		return 1.0 - std::exp( -ratePerHour * dt / 3600.0 );
	}

	/** Probability 1 - exp(-excess / scale) of a detonation, 0 inside the limit, DETONATION_MAX at most. */
	inline double ExcessProbability( double excess, double scale )
	{
		if (excess <= 0.0) return 0.0;
		const double p = 1.0 - exp( -excess / scale );
		return (std::min)( DETONATION_MAX, p );
	}

	/**
	 * Probability that the hydrazine detonates when an APU is started (or the fuel reaches the gas generator) with these temperatures:
	 * zero inside the limits, growing above them (injector 400 F: 28% at 450 F, 73% at 600 F; the pump and the valve module have a narrower scale).
	 */
	inline double DetonationProbability( const Thermal& t )
	{
		double p = ExcessProbability( t.injector - LIMIT_INJECTOR_F, DETONATION_SCALE_INJECTOR_F );
		p = (std::max)( p, ExcessProbability( t.pump - LIMIT_PUMP_F, DETONATION_SCALE_PUMP_F ) );
		p = (std::max)( p, ExcessProbability( t.valve - LIMIT_VALVE_F, DETONATION_SCALE_PUMP_F ) );
		return p;
	}

	/** Relative rate of a leak of a failure from a normal random number (mean 0, sigma 1): 1 +- 30%, between 0.5 and 1.8. */
	inline double LeakFactor( double gauss )
	{
		return (std::max)( 0.5, (std::min)( 1.8, 1.0 + (LEAK_SIGMA * gauss) ) );
	}

	/** Rate of the leak [lb/s] from the factor of the failure and the variation of this second (normal random number). */
	inline double LeakRate( double factor, double jitterGauss )
	{
		return LEAK_RATE * factor * (std::max)( 0.7, (std::min)( 1.3, 1.0 + (LEAK_JITTER * jitterGauss) ) );
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
