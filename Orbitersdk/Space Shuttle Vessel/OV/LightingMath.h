/****************************************************************************
  This file is part of Space Shuttle Vessel

  Cabin lighting: the levels of the floodlights and of the panel lighting from the position of the switches and of the DIM/BRT knobs
  (no Orbiter calls, so it can be tested alone)

  The real lighting system (Crew Operations Manual, 2.15): the floodlights of the glareshields have a BRIGHT, VAR, OFF switch and a
  DIM/BRT knob that works in the VAR position; the seat/center console lights have a SEAT, OFF, CTR CNSL switch and a DIM/BRT knob; the
  mission, payload and orbit stations have an ON/OFF switch and a DIM/BRT knob; the panels have OFF/BRT knobs.


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
#ifndef _LIGHTINGMATH_H_
#define _LIGHTINGMATH_H_


#include <cmath>


namespace lightmath
{
	inline constexpr double KNOB_VOLTS = 5.0;// the knobs give 5 V at the BRT stop

	/** Position of a knob [0, 1] from the voltage of its line. */
	inline double KnobFromVolts( double volts )
	{
		const double k = volts / KNOB_VOLTS;
		return (k < 0.0) ? 0.0 : ((k > 1.0) ? 1.0 : k);
	}

	/** Level of a lamp from the position of its DIM/BRT knob: the eye follows roughly a square law, so the low end of the knob has the finer steps. */
	inline double KnobLevel( double knob )
	{
		const double k = (knob < 0.0) ? 0.0 : ((knob > 1.0) ? 1.0 : knob);
		return k * k;
	}

	/** Floodlight of a glareshield: BRIGHT is the full level, VAR follows the knob, OFF is dark (BRIGHT has priority if two positions are read). */
	inline double GlareshieldLevel( bool bright, bool var, double knob )
	{
		if (bright) return 1.0;
		if (var) return KnobLevel( knob );
		return 0.0;
	}

	/** Floodlight with an ON/OFF switch and a DIM/BRT knob (mission, payload and orbit stations). */
	inline double SwitchedLevel( bool on, double knob )
	{
		return on ? KnobLevel( knob ) : 0.0;
	}

	enum SeatConsoleMode {SC_OFF = 0, SC_SEAT, SC_CONSOLE};

	/** Light of the seat and the center console: the lamp that is lit is the one that the switch selects (both or none read: off). */
	inline SeatConsoleMode SeatConsoleSelect( bool seat, bool console )
	{
		if (seat && !console) return SC_SEAT;
		if (console && !seat) return SC_CONSOLE;
		return SC_OFF;
	}

	/** Emergency lighting (Crew Operations Manual 2.15): the relay of the emergency system puts the emergency fixtures at full bright, whatever their controls say. */
	inline double EmergencyLevel( double level, bool emergency )
	{
		return emergency ? 1.0 : level;
	}

	/**
	 * Self illumination of the panels (1 = as in the meshes) from the option for the lamps off and how much the cabin is lit (0 = lamps off, 1 = lit):
	 * the panels follow the cabin, so they do not shine in a dark cabin.
	 */
	inline double PanelGlow( double glowLampsOff, double cabinLit )
	{
		const double g = (glowLampsOff < 0.0) ? 0.0 : ((glowLampsOff > 1.0) ? 1.0 : glowLampsOff);
		const double c = (cabinLit < 0.0) ? 0.0 : ((cabinLit > 1.0) ? 1.0 : cabinLit);
		return g + ((1.0 - g) * c);
	}

	/** Level of the middeck floodlights from the option: 0 = all off, 1 = half of the fixtures (half of the light), 2 = all on. */
	inline double MiddeckLevel( int choice )
	{
		return (choice <= 0) ? 0.0 : ((choice == 1) ? 0.5 : 1.0);
	}

	/** Multiplier of the self illumination of a panel from its OFF/BRT knob (1 at BRT: the panel looks as without the lighting system). */
	inline double PanelScale( double knob )
	{
		return KnobLevel( knob );
	}

	/** Intensity of a light emitter from the levels of the lamp and the power of the fixture, limited to [0, 1]. */
	inline double EmitterIntensity( double level, double fixturePower )
	{
		const double v = level * fixturePower;
		return (v < 0.0) ? 0.0 : ((v > 1.0) ? 1.0 : v);
	}

	/** Moves a level to the target at a limited rate (lamps do not jump: the fluorescent lamps of the floodlights have a short ramp). */
	inline double Ramp( double level, double target, double rate, double dt )
	{
		const double step = rate * dt;
		if (target > (level + step)) return level + step;
		if (target < (level - step)) return level - step;
		return target;
	}
}


#endif// _LIGHTINGMATH_H_
