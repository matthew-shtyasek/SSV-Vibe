/****************************************************************************
  This file is part of Space Shuttle Vessel

  Heating and cooling of the hydraulic fluid and of the APU lube oil, with the water spray boiler (WSB) that cools them, and what happens
  when they overheat (no Orbiter calls, so it can be tested alone)

  Data of the Crew Operations Manual (USA007587, 2.1 and 6.8):
  - the boiler keeps the lube oil near 250 F and the hydraulic fluid between 210 and 220 F; the water feed valve of the hydraulic fluid opens
    around the set point of 208 F, the bypass valve sends the fluid through the boiler at 210 F and around it at 190 F
  - the boiler has 142 lb of water, the spray bars give at most 10 lb/min to the hydraulic fluid and 5 lb/min to the lube oil, and the
    nitrogen tank pushes the water (it has to have pressure to spray)
  - excessive temperatures are more than 230 F in the hydraulic fluid (caution, HYD RSVR T) and more than 290 F in the lube oil (APU TEMP);
    the hydraulic seals degrade above the limits, and with no cooling of the lube oil an APU has only 2 to 3 minutes (4 to 5 in another
    table) after it reached the full operating temperature before the bearings seize
  - if the temperature goes up, the first action is to change the controller of the boiler (A or B)
  The heating rates and the heat capacities are not published: they are chosen so that the times of the manual agree (the lube oil of a
  running APU reaches 250 F in about 15 minutes, a lube oil without cooling at 250 F needs about 3 minutes to reach 290 F).


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
#ifndef _HYDMODEL_H_
#define _HYDMODEL_H_


#include <cmath>


namespace hydmodel
{
	// temperatures [F]
	inline constexpr double AMBIENT = 70.0;
	inline constexpr double OIL_SETPOINT = 250.0;
	inline constexpr double OIL_BAND = 8.0;// the spray of the lube oil goes from 0 to full in this range above the set point
	inline constexpr double OIL_LIMIT = 290.0;// excessive lube oil temperature
	inline constexpr double HYD_SETPOINT = 208.0;
	inline constexpr double HYD_BAND = 6.0;
	inline constexpr double HYD_THROUGH = 210.0;// the fluid is sent through the boiler from here
	inline constexpr double HYD_BYPASS = 190.0;// and around it from here
	inline constexpr double HYD_CAUTION = 230.0;// excessive hydraulic fluid temperature (HYD RSVR T)
	inline constexpr double HYD_SEAL_LIMIT = 250.0;// the seals degrade above this temperature

	// heating
	inline constexpr double OIL_HEAT = 0.30;// [F/s] lube oil of a running APU at full speed
	inline constexpr double OIL_AMBIENT_TAU = 3000.0;// [s] loss to the surroundings of the lube oil
	inline constexpr double OIL_STOPPED_TAU = 1800.0;// [s] cooling of the lube oil of a stopped APU
	inline constexpr double HYD_RISE_MAX = 185.0;// [F] rise of the fluid above the ambient at full load, nominal pressure, with no cooling
	inline constexpr double HYD_HEAT_TAU = 900.0;// [s]
	inline constexpr double HYD_STOPPED_TAU = 2400.0;// [s]
	inline constexpr double LOW_PRESSURE_HEAT = 0.55;// share of the heat with the pump at low pressure

	// cooling of the boiler
	inline constexpr double OIL_COOL_MAX = 1.0;// [F/s] with the spray bars of the lube oil at full flow
	inline constexpr double HYD_COOL_MAX = 0.28;// [F/s] with the spray bars of the hydraulic fluid at full flow
	inline constexpr double WATER_TANK = 142.0;// [lb]
	inline constexpr double OIL_SPRAY = 5.0;// [lb/min] at full flow
	inline constexpr double HYD_SPRAY = 10.0;// [lb/min] at full flow
	inline constexpr double N2_SPRAY_MIN = 0.30;// the spray works with a nitrogen pressure above this share of the nominal (below, less)

	// what the heat does
	inline constexpr double OIL_SEIZE_TIME = 60.0;// [s] above the limit of the lube oil before the bearings seize
	inline constexpr double OIL_SEIZE_TEMP = 340.0;// [F] the bearings seize at once
	inline constexpr double HYD_DAMAGE_TIME = 300.0;// [s] above 260 F (10 F over the limit) that the seals need to be damaged
	inline constexpr double HYD_LEAK_MAX = 1.0 / 600.0;// [1/s] of the reservoir with the seals damaged (the reservoir is empty in 10 minutes)
	inline constexpr double HYD_QTY_MIN_OK = 0.05;// the pump can not hold the pressure with less of the reservoir

	struct State
	{
		double hyd;// temperature of the hydraulic fluid [F]
		double oil;// temperature of the lube oil [F]
		double water;// water in the tank of the boiler [lb]
		double n2;// nitrogen of the tank of the boiler [0, 1]
		bool through;// the hydraulic fluid goes through the boiler (not around it)
		double hydDamage;// damage of the seals [0, 1]
		double oilOver;// time of the lube oil above the limit [s]
		double hydQty;// quantity of the reservoir [0, 1]
		bool seized;// the bearings of the APU seized
		// failures
		bool ctrlFail[2];// controller A, B
		bool iceHyd, iceOil;// the spray valve of the hydraulic fluid, or of the lube oil, is frozen
		double n2Leak;// [1/s] of the nitrogen
		double hydLeak;// [1/s] of the reservoir (a leak of the system, apart from the seals)
		// results of the last step
		double hydDuty, oilDuty;// spray of the boiler [0, 1] (share of the flow)
	};

	struct Input
	{
		double speed;// [0, 1] of the APU (1 = 100%); 0 = stopped
		bool pumpNorm;// the main pump at nominal pressure (not low)
		double load;// [0, 1] load of the hydraulic system
		double ambient;// [F]
		bool powered;// BOILER PWR on
		int controller;// 0 = none, 1 = A, 2 = B
		bool n2Open;// BOILER N2 SUPPLY open
	};

	inline State InitialState( void )
	{
		State s;
		s.hyd = AMBIENT;
		s.oil = AMBIENT;
		s.water = WATER_TANK;
		s.n2 = 1.0;
		s.through = false;
		s.hydDamage = 0.0;
		s.oilOver = 0.0;
		s.hydQty = 1.0;
		s.seized = false;
		s.ctrlFail[0] = s.ctrlFail[1] = false;
		s.iceHyd = s.iceOil = false;
		s.n2Leak = 0.0;
		s.hydLeak = 0.0;
		s.hydDuty = s.oilDuty = 0.0;
		return s;
	}

	inline double Clamp01( double x )
	{
		return (x < 0.0) ? 0.0 : ((x > 1.0) ? 1.0 : x);
	}

	/** The controller that is selected is powered and has not failed. */
	inline bool ControllerWorks( const State& s, const Input& in )
	{
		if (!in.powered) return false;
		if ((in.controller != 1) && (in.controller != 2)) return false;
		return !s.ctrlFail[in.controller - 1];
	}

	/** Position of the bypass valve of the hydraulic fluid, with the hysteresis (through the boiler from 210 F, around it from 190 F). */
	inline bool BypassLogic( bool through, double hyd )
	{
		if (hyd >= HYD_THROUGH) return true;
		if (hyd <= HYD_BYPASS) return false;
		return through;
	}

	/** Spray of the hydraulic fluid [0, 1] that the controller asks for. */
	inline double HydSprayCommand( double hyd )
	{
		return Clamp01( (hyd - HYD_SETPOINT) / HYD_BAND );
	}

	/** Spray of the lube oil [0, 1] that the controller asks for. */
	inline double OilSprayCommand( double oil )
	{
		return Clamp01( (oil - OIL_SETPOINT) / OIL_BAND );
	}

	/** Share of the flow of the spray with the nitrogen of the tank (the water is pushed by the nitrogen). */
	inline double N2Effect( double n2, bool open )
	{
		if (!open) return 0.0;
		return Clamp01( n2 / N2_SPRAY_MIN );
	}

	/** Temperature that the hydraulic fluid would have with no cooling at the load and the speed (steady state). */
	inline double HydSteadyNoCooling( const Input& in )
	{
		const double load = Clamp01( in.load );
		return in.ambient + (HYD_RISE_MAX * (0.25 + (0.75 * load)) * (in.pumpNorm ? 1.0 : LOW_PRESSURE_HEAT) * Clamp01( in.speed ));
	}

	/** True if the hydraulic system can hold the pressure (the reservoir has fluid). */
	inline bool HydraulicsOk( const State& s )
	{
		return s.hydQty > HYD_QTY_MIN_OK;
	}

	/** Advances the state dt seconds. */
	inline void Step( State& s, const Input& in, double dt )
	{
		if (dt <= 0.0) return;
		const double speed = Clamp01( in.speed );

		// heating, or the cooling of the stopped system by the surroundings
		if (speed > 0.02)
		{
			s.hyd += (HydSteadyNoCooling( in ) - s.hyd) * (dt / HYD_HEAT_TAU);
			s.oil += ((OIL_HEAT * speed) - ((s.oil - in.ambient) / OIL_AMBIENT_TAU)) * dt;
		}
		else
		{
			s.hyd += (in.ambient - s.hyd) * (dt / HYD_STOPPED_TAU);
			s.oil += (in.ambient - s.oil) * (dt / OIL_STOPPED_TAU);
		}

		// the boiler
		const bool ctrl = ControllerWorks( s, in );
		if (ctrl) s.through = BypassLogic( s.through, s.hyd );
		const double n2eff = N2Effect( s.n2, in.n2Open );
		const bool water = (s.water > 0.0);
		s.hydDuty = (ctrl && s.through && !s.iceHyd && water) ? (HydSprayCommand( s.hyd ) * n2eff) : 0.0;
		s.oilDuty = (ctrl && !s.iceOil && water) ? (OilSprayCommand( s.oil ) * n2eff) : 0.0;
		s.hyd -= HYD_COOL_MAX * s.hydDuty * dt;
		s.oil -= OIL_COOL_MAX * s.oilDuty * dt;
		if (s.hyd < in.ambient) s.hyd = in.ambient;
		if (s.oil < in.ambient) s.oil = in.ambient;
		s.water -= ((HYD_SPRAY * s.hydDuty) + (OIL_SPRAY * s.oilDuty)) * dt / 60.0;
		if (s.water < 0.0) s.water = 0.0;
		s.n2 -= s.n2Leak * dt;
		if (s.n2 < 0.0) s.n2 = 0.0;

		// the lube oil: above the limit the bearings seize
		if (s.oil > OIL_LIMIT) s.oilOver += dt;
		else
		{
			s.oilOver -= 0.5 * dt;
			if (s.oilOver < 0.0) s.oilOver = 0.0;
		}
		if ((s.oilOver >= OIL_SEIZE_TIME) || (s.oil >= OIL_SEIZE_TEMP)) s.seized = true;

		// the hydraulic fluid: the seals degrade above their limit, and the system leaks
		if (s.hyd > HYD_SEAL_LIMIT)
		{
			s.hydDamage += ((s.hyd - HYD_SEAL_LIMIT) / 10.0) * (dt / HYD_DAMAGE_TIME);
			if (s.hydDamage > 1.0) s.hydDamage = 1.0;
		}
		const double leak = (HYD_LEAK_MAX * s.hydDamage) + s.hydLeak;
		s.hydQty -= leak * dt;
		if (s.hydQty < 0.0) s.hydQty = 0.0;
	}

	/** Caution of the temperature of the hydraulic fluid (HYD RSVR T). */
	inline bool HydCaution( const State& s )
	{
		return s.hyd > HYD_CAUTION;
	}

	/** Caution of the temperature of the lube oil (APU TEMP). */
	inline bool OilCaution( const State& s )
	{
		return s.oil > OIL_LIMIT;
	}

	/** Volts of a sensor of temperature (0 to 5 V for 0 to 500 F). */
	inline double TemperatureToVolts( double f )
	{
		const double v = f / 100.0;
		return (v < 0.0) ? 0.0 : ((v > 5.0) ? 5.0 : v);
	}

	/** Volts of the sensor of the hydraulic fluid that goes to the C&W: the limit of HYD RSVR T (230 F) is 4.2 V of the table of the C&W. */
	inline double HydTemperatureToCwVolts( double f )
	{
		const double v = f * (4.2 / HYD_CAUTION);
		return (v < 0.0) ? 0.0 : ((v > 5.0) ? 5.0 : v);
	}

	/** Temperature from the volts of the sensor. */
	inline double VoltsToTemperature( double v )
	{
		return v * 100.0;
	}

	/** Load of the hydraulic system from the dynamic pressure [Pa] (the aerosurfaces) and the engines: 0.25 with the vehicle in orbit. */
	inline double LoadFromFlight( double dynPressure, bool mainEnginesRunning )
	{
		double load = 0.25 + (0.75 * Clamp01( dynPressure / 3000.0 ));
		if (mainEnginesRunning && (load < 0.7)) load = 0.7;// the gimbals of the engines
		return load;
	}
}


#endif// _HYDMODEL_H_
