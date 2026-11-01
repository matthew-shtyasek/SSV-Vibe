/****************************************************************************
  This file is part of Space Shuttle Vessel

  Table of the options of the vessel (the ones of Config\Vessels\SSV_OV.cfg, that the settings MFD shows and changes) and the operations on
  their values (no Orbiter calls, so it can be tested alone)


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
#ifndef _SETTINGSMODEL_H_
#define _SETTINGSMODEL_H_


#include <cmath>
#include <cstdio>
#include <cstring>


namespace settings
{
	enum Kind {KIND_BOOL = 0, KIND_INT, KIND_FLOAT, KIND_CHOICE};

	// the first three groups are on the page SETTINGS, the others on the page REALISM (failures, tolerances, noise, mass effects)
	enum Group {GROUP_FLIGHT = 0, GROUP_RMS, GROUP_CABIN, GROUP_MASS, GROUP_HYD, GROUP_RCS, GROUP_COUNT};
	inline constexpr int FIRST_REALISM_GROUP = GROUP_MASS;
	inline bool IsRealismGroup( int group ) { return group >= FIRST_REALISM_GROUP; }

	/** One option. The key is the name in the configuration files. */
	struct Item
	{
		const char* key;
		const char* label;// short, for the MFD
		const char* note;// what it does
		int kind;
		double def;// default value (a boolean is 0 or 1)
		double min;
		double max;
		double step;
		bool restart;// the option is read only when the vessel is created: the change is for the next start of the simulation
		int group;
		const char* const* choices;// names of the values of a KIND_CHOICE (the value is the index)
	};

	/** Keys (with Ctrl) that switch the free movement of the head on and off. */
	inline constexpr const char* const FREE_MOVE_KEYS[] = {"Ctrl+Home", "Ctrl+End", "Ctrl+Insert", "Ctrl+Delete", "Ctrl+Scroll Lock", "Ctrl+Grave (`)"};
	inline constexpr int FREE_MOVE_KEY_COUNT = sizeof( FREE_MOVE_KEYS ) / sizeof( FREE_MOVE_KEYS[0] );

	/** Random failures of the water spray boilers: names of the choices and the rate of each [1/hour per boiler]. */
	inline constexpr const char* const HYD_FAILURE_RATE_NAMES[] = {"Off", "1 in 20 h", "1 in 5 h", "1 in 1 h"};
	inline constexpr double HYD_FAILURE_RATES[] = {0.0, 0.05, 0.2, 1.0};
	inline constexpr int HYD_FAILURE_RATE_COUNT = sizeof( HYD_FAILURE_RATES ) / sizeof( HYD_FAILURE_RATES[0] );

	/** Time acceleration from which the orbit DAP holds the attitude on rails, without firing the jets (see RCSMath.h): names of the choices and the values. */
	inline constexpr const char* const RCS_RAILS_NAMES[] = {"Off", "100x", "300x", "1000x", "3000x"};
	inline constexpr double RCS_RAILS_ACC[] = {0.0, 100.0, 300.0, 1000.0, 3000.0};
	inline constexpr int RCS_RAILS_COUNT = sizeof( RCS_RAILS_ACC ) / sizeof( RCS_RAILS_ACC[0] );

	/** Middeck floodlights (the panel MO13Q with their switches is not in the virtual cockpit). */
	inline constexpr const char* const MIDDECK_LIGHT_NAMES[] = {"Off", "Half", "All"};

	/** Index of each option in the table (the order of the table). */
	enum Index
	{
		AUTO_ACTION_LANDING_GEAR = 0,
		AUTO_ACTION_DRAG_CHUTE,
		ALARM_1X,
		POSITION_LABEL_TIME,
		ITEM_RHC_RATE,
		ITEM_RPTA_RATE,
		ITEM_BRAKE_RATE,
		EIU_DATA_RECORDER,
		RCS_RAILS_ABOVE,
		FLIGHT_DATA_LOG,
		RMS_PAYLOAD_MASS_EFFECT,
		RMS_VIA_GPC,
		CABIN_LIGHTING,
		KNOB_TOP_OFFSET,
		KNOB_TOP_TRAVEL,
		KNOB_TOP_REVERSE,
		KNOB_AFT_OFFSET,
		KNOB_AFT_TRAVEL,
		KNOB_AFT_REVERSE,
		KNOB_L9_OFFSET,
		KNOB_L9_TRAVEL,
		KNOB_L9_REVERSE,
		KNOB_R10_OFFSET,
		KNOB_R10_TRAVEL,
		KNOB_R10_REVERSE,
		VC_FREE_MOVE,
		VC_FREE_MOVE_KEY,
		MIDDECK_LIGHTS,
		PANEL_GLOW,
		RMS_REACTION,
		RMS_CENTER_OF_MASS,
		RMS_INERTIA,
		HYDRAULIC_THERMAL,
		HYDRAULIC_RANDOM_FAILURES,
		APU_RANDOM_FIRE,
		HYDRAULIC_TOLERANCES,
		HYDRAULIC_NOISE,
		RCS_TOLERANCES,
		RCS_NOISE,
		TOLERANCE_SCALE,
		COUNT
	};

	inline constexpr Item ITEMS[] = {
		{"AutoActionLandingGear", "Auto landing gear", "The landing gear goes down by itself at the programmed altitude", KIND_BOOL, 1.0, 0.0, 1.0, 1.0, false, GROUP_FLIGHT},
		{"AutoActionDragChute", "Auto drag chute", "The drag chute is deployed and jettisoned by itself", KIND_BOOL, 1.0, 0.0, 1.0, 1.0, false, GROUP_FLIGHT},
		{"Alarm1x", "Alarm tone 1x", "Sound of the master alarm every time, not only the first", KIND_BOOL, 1.0, 0.0, 1.0, 1.0, false, GROUP_FLIGHT},
		{"PositionLabelTime", "Station label time [s]", "Time that the name of the station is shown on the screen (0 = not shown)", KIND_INT, 3.0, 0.0, 60.0, 1.0, false, GROUP_FLIGHT},
		{"RHCRate", "RHC key rate", "Rate of the keyboard hand controller (1/x s from the detent to full deflection)", KIND_FLOAT, 0.75, 0.05, 5.0, 0.05, false, GROUP_FLIGHT},
		{"RPTARate", "Pedals key rate", "Rate of the keyboard rudder pedals (1/x s from the detent to full deflection)", KIND_FLOAT, 0.75, 0.05, 5.0, 0.05, false, GROUP_FLIGHT},
		{"BrakeRate", "Brakes key rate", "Rate of the keyboard toe brakes (1/x s from the detent to full deflection)", KIND_FLOAT, 0.75, 0.05, 5.0, 0.05, false, GROUP_FLIGHT},
		{"EIUDataRecorder", "EIU data recorder", "Writes the data of the engine interface units to files (for debugging)", KIND_BOOL, 0.0, 0.0, 1.0, 1.0, true, GROUP_FLIGHT},
		{"RCSRailsAbove", "DAP on rails from", "From this time acceleration the DAP holds the attitude without the jets (mean propellant use subtracted)", KIND_CHOICE, 2.0, 0.0, RCS_RAILS_COUNT - 1, 1.0, false, GROUP_FLIGHT, RCS_RAILS_NAMES},
		{"FlightDataLog", "Flight data log (debug)", "Saves data of the DAP and of the hydraulics (APU, surfaces, brakes) to the folder SSV_FlightLogs", KIND_BOOL, 1.0, 0.0, 1.0, 1.0, false, GROUP_FLIGHT},
		{"RMSPayloadMassEffect", "Payload slows the arm", "The rates of the arm decrease with the mass of the payload that it holds", KIND_BOOL, 0.0, 0.0, 1.0, 1.0, false, GROUP_RMS},
		{"RMSViaGPC", "RMS control via GPC", "Modes of the arm run in the GPC software (SM OPS 2); without it only DIRECT and BACKUP work", KIND_BOOL, 1.0, 0.0, 1.0, 1.0, false, GROUP_RMS},
		{"CabinLighting", "Cabin lighting", "Floodlights and panel lighting follow the switches and knobs (needs the local lights of Orbiter)", KIND_BOOL, 1.0, 0.0, 1.0, 1.0, false, GROUP_CABIN},
		{"KnobTopOffset", "Knobs O6/O8: pointer offset", "Pointer of the mesh at rest from the middle of the travel, deg (CCW +); live", KIND_FLOAT, 0.0, -360.0, 360.0, 5.0, false, GROUP_CABIN},
		{"KnobTopTravel", "Knobs O6/O8: travel", "Rotation between DIM/OFF and BRT, deg; live", KIND_FLOAT, 300.0, 90.0, 360.0, 5.0, false, GROUP_CABIN},
		{"KnobTopReverse", "Knobs O6/O8: reversed", "Reverses the direction of the pointer; live", KIND_BOOL, 1.0, 0.0, 1.0, 1.0, false, GROUP_CABIN},
		{"KnobAftOffset", "Knobs A6U: pointer offset", "Pointer of the mesh at rest from the middle of the travel, deg (CCW +); live", KIND_FLOAT, 150.0, -360.0, 360.0, 5.0, false, GROUP_CABIN},
		{"KnobAftTravel", "Knobs A6U: travel", "Rotation between DIM/OFF and BRT, deg; live", KIND_FLOAT, 300.0, 90.0, 360.0, 5.0, false, GROUP_CABIN},
		{"KnobAftReverse", "Knobs A6U: reversed", "Reverses the direction of the pointer; live", KIND_BOOL, 0.0, 0.0, 1.0, 1.0, false, GROUP_CABIN},
		{"KnobL9Offset", "Knobs L9: pointer offset", "Pointer of the mesh at rest from the middle of the travel, deg (CCW +); live", KIND_FLOAT, 40.0, -360.0, 360.0, 5.0, false, GROUP_CABIN},
		{"KnobL9Travel", "Knobs L9: travel", "Rotation between DIM/OFF and BRT, deg; live", KIND_FLOAT, 240.0, 90.0, 360.0, 5.0, false, GROUP_CABIN},
		{"KnobL9Reverse", "Knobs L9: reversed", "Reverses the direction of the pointer; live", KIND_BOOL, 1.0, 0.0, 1.0, 1.0, false, GROUP_CABIN},
		{"KnobR10Offset", "Knobs R10: pointer offset", "Pointer of the mesh at rest from the middle of the travel, deg (CCW +); live", KIND_FLOAT, -40.0, -360.0, 360.0, 5.0, false, GROUP_CABIN},
		{"KnobR10Travel", "Knobs R10: travel", "Rotation between DIM/OFF and BRT, deg; live", KIND_FLOAT, 265.0, 90.0, 360.0, 5.0, false, GROUP_CABIN},
		{"KnobR10Reverse", "Knobs R10: reversed", "Reverses the direction of the pointer; live", KIND_BOOL, 1.0, 0.0, 1.0, 1.0, false, GROUP_CABIN},
		{"VCFreeMove", "Free head movement", "A key switches it on: then the arrows and Ins/Del move the head around the station", KIND_BOOL, 1.0, 0.0, 1.0, 1.0, false, GROUP_CABIN},
		{"VCFreeMoveKey", "Free movement key", "Key that switches the free movement of the head on and off (the arrows are used by the THC)", KIND_CHOICE, 0.0, 0.0, FREE_MOVE_KEY_COUNT - 1, 1.0, false, GROUP_CABIN, FREE_MOVE_KEYS},
		{"MiddeckLights", "Middeck floodlights", "Floodlights of the middeck (panel MO13Q is not in the cockpit)", KIND_CHOICE, 2.0, 0.0, 2.0, 1.0, false, GROUP_CABIN, MIDDECK_LIGHT_NAMES},
		{"PanelGlow", "Panel glow, lamps off", "Self illumination of the panels when the cabin lamps are off (1 = as in the meshes, 0 = none)", KIND_FLOAT, 0.5, 0.0, 1.0, 0.05, false, GROUP_CABIN},
		{"RMSReaction", "Arm pushes the orbiter", "Reaction forces of the arm and of its payload on the orbiter (it turns with the DAP in FREE)", KIND_BOOL, 0.0, 0.0, 1.0, 1.0, false, GROUP_MASS},
		{"RMSCenterOfMass", "C.g. follows the arm", "The center of mass of the orbiter follows the arm and the payload that it holds", KIND_BOOL, 0.0, 0.0, 1.0, 1.0, false, GROUP_MASS},
		{"RMSInertia", "Inertia with the arm", "The moments of inertia of the orbiter change with the arm and the payload that it holds", KIND_BOOL, 0.0, 0.0, 1.0, 1.0, false, GROUP_MASS},
		{"HydraulicThermal", "Hydraulic heating", "Heat of the hydraulic fluid and lube oil, boiler cooling and overheating failures", KIND_BOOL, 1.0, 0.0, 1.0, 1.0, false, GROUP_HYD},
		{"HydraulicRandomFailures", "Random boiler failures", "Random boiler failures (to train): dice thrown at random 5-25 min intervals; rate per boiler", KIND_CHOICE, 0.0, 0.0, HYD_FAILURE_RATE_COUNT - 1, 1.0, false, GROUP_HYD, HYD_FAILURE_RATE_NAMES},
		{"APURandomFire", "Random APU fire (like STS-9)", "A working APU can leak and burn in the air (to train); rate per APU, per hour of running in the air", KIND_CHOICE, 0.0, 0.0, HYD_FAILURE_RATE_COUNT - 1, 1.0, false, GROUP_HYD, HYD_FAILURE_RATE_NAMES},
		{"HydraulicTolerances", "Tolerances of the parts", "Hydraulic parts of this vehicle differ a little (pressure, set points, heat); fixed, saved with it", KIND_BOOL, 0.0, 0.0, 1.0, 1.0, false, GROUP_HYD},
		{"HydraulicNoise", "Noise of each use", "Hydraulics: values vary a little at each use (quick variations over the tolerances)", KIND_BOOL, 0.0, 0.0, 1.0, 1.0, false, GROUP_HYD},
		{"RCSTolerances", "Tolerances of the jets", "Each RCS jet differs a little (thrust, Isp, delays, angles, offsets); fixed, saved with it", KIND_BOOL, 0.0, 0.0, 1.0, 1.0, false, GROUP_RCS},
		{"RCSNoise", "Noise of each firing", "RCS: thrust, Isp and delay vary a little at each firing (not the installation)", KIND_BOOL, 0.0, 0.0, 1.0, 1.0, false, GROUP_RCS},
		{"ToleranceScale", "Scale of the dispersions", "Multiplies the tolerances and the noise of all the systems (1.0 = estimated real values)", KIND_FLOAT, 1.0, 0.0, 3.0, 0.1, false, GROUP_RCS},
	};
	static_assert( (sizeof( ITEMS ) / sizeof( ITEMS[0] )) == COUNT, "the table of the options and its indices" );

	inline const char* GroupName( int group )
	{
		switch (group)
		{
			case GROUP_FLIGHT: return "FLIGHT";
			case GROUP_RMS: return "RMS";
			case GROUP_CABIN: return "CABIN";
			case GROUP_MASS: return "MASS";
			case GROUP_HYD: return "HYD";
			case GROUP_RCS: return "RCS";
		}
		return "";
	}

	/** Limits a value to the range of the option; an integer or a boolean is rounded. */
	inline double Clamp( int i, double v )
	{
		const Item& it = ITEMS[i];
		if (it.kind != KIND_FLOAT) v = floor( v + 0.5 );
		if (v < it.min) v = it.min;
		if (v > it.max) v = it.max;
		return v;
	}

	/**
	 * Changes a value one step up (dir > 0) or down (dir < 0). A boolean is toggled (in any direction). The value is kept on the grid of
	 * the step, so it does not accumulate the error of the sums (0.75 + 0.05 is 0.8 and not 0.8000000000000002).
	 */
	inline double Step( int i, double v, int dir )
	{
		const Item& it = ITEMS[i];
		if (it.kind == KIND_BOOL) return (v != 0.0) ? 0.0 : 1.0;
		if (it.kind == KIND_CHOICE)
		{
			// the list is circular
			const int n = (int)(it.max - it.min) + 1;
			const int i0 = (int)floor( v + 0.5 );
			return it.min + (double)((((i0 - (int)it.min) + ((dir >= 0) ? 1 : (n - 1))) % n + n) % n);
		}
		const double n = floor( ((v - it.min) / it.step) + 0.5 ) + ((dir > 0) ? 1.0 : ((dir < 0) ? -1.0 : 0.0));
		double r = it.min + (n * it.step);
		if (it.kind == KIND_FLOAT) r = floor( (r * 1e9) + 0.5 ) / 1e9;// no digits of the error of the product
		return Clamp( i, r );
	}

	inline bool IsDefault( int i, double v )
	{
		return fabs( v - ITEMS[i].def ) < 1e-9;
	}

	/** Text of a value for the MFD. */
	inline void FormatValue( int i, double v, char* buf, size_t size )
	{
		const Item& it = ITEMS[i];
		if (it.kind == KIND_BOOL) sprintf_s( buf, size, "%s", (v != 0.0) ? "ON" : "OFF" );
		else if (it.kind == KIND_INT) sprintf_s( buf, size, "%d", (int)floor( v + 0.5 ) );
		else if (it.kind == KIND_CHOICE) sprintf_s( buf, size, "%s", it.choices[(int)floor( v + 0.5 )] );
		else sprintf_s( buf, size, "%.2f", v );
	}

	/** @return the index of the option with the key (not case sensitive), or -1 */
	inline int FindKey( const char* key )
	{
		for (int i = 0; i < COUNT; i++)
		{
			if (_stricmp( key, ITEMS[i].key ) == 0) return i;
		}
		return -1;
	}

	// failures of the hydraulic systems that the MFD can set (3 systems with 6 failures each)
	inline constexpr int FAIL_SYSTEMS = 3;
	inline constexpr int FAIL_KINDS = 6;
	inline constexpr int FAIL_ROWS = FAIL_SYSTEMS * FAIL_KINDS;
	inline int FailSystem( int row ) { return (row / FAIL_KINDS) + 1; }
	inline int FailKind( int row ) { return row % FAIL_KINDS; }

	/** Pages of the MFD */
	enum Page {PAGE_SETTINGS = 0, PAGE_REALISM, PAGE_FAILURES, PAGE_STATUS, PAGE_CONTROLLERS, PAGE_COUNT};
	inline int NextPage( int page, int dir ) { return (page + ((dir >= 0) ? 1 : (PAGE_COUNT - 1))) % PAGE_COUNT; }

	/** Position of the option in the list of its group (the MFD shows the groups one after the other). */
	inline int CountInGroup( int group )
	{
		int n = 0;
		for (int i = 0; i < COUNT; i++)
		{
			if (ITEMS[i].group == group) n++;
		}
		return n;
	}

	/**
	 * State of the lists of the settings MFD: the values, the line that is selected and what changed. There are two lists (views), the one of
	 * the page SETTINGS and the one of the page REALISM; each has its own selected line. The values of the editor are applied by the MFD to
	 * the options of the vessel.
	 */
	class Editor
	{
	public:
		Editor( void )
		{
			realism = false;
			cursors[0] = FirstOf( false );
			cursors[1] = FirstOf( true );
			dirty = false;
			for (int i = 0; i < COUNT; i++) values[i] = ITEMS[i].def;
		}

		/** Selects the list: false = SETTINGS, true = REALISM. */
		void SetRealism( bool r ) { realism = r; }
		bool IsRealism( void ) const { return realism; }

		void SetValues( const double* v )
		{
			for (int i = 0; i < COUNT; i++) values[i] = Clamp( i, v[i] );
			dirty = false;
		}

		double Value( int i ) const { return values[i]; }
		int Cursor( void ) const { return cursors[realism ? 1 : 0]; }

		/** True if the option is in the list that is shown. */
		bool Visible( int i ) const { return IsRealismGroup( ITEMS[i].group ) == realism; }

		/** Number of lines of the list that is shown. */
		int Lines( void ) const
		{
			int n = 0;
			for (int i = 0; i < COUNT; i++) n += Visible( i ) ? 1 : 0;
			return n;
		}

		/** Selected lines of the two lists and the mark of the unsaved change: kept when the MFD is created again (a change of its size). */
		void GetState( int* cur, bool* d ) const { cur[0] = cursors[0]; cur[1] = cursors[1]; *d = dirty; }
		void SetState( const int* cur, bool d )
		{
			for (int l = 0; l < 2; l++)
			{
				if ((cur[l] >= 0) && (cur[l] < COUNT) && (IsRealismGroup( ITEMS[cur[l]].group ) == (l == 1))) cursors[l] = cur[l];
			}
			dirty = d;
		}

		/** True after a change that was not written to the file. */
		bool IsDirty( void ) const { return dirty; }
		void MarkSaved( void ) { dirty = false; }

		/** Moves the selected line (the list is circular). */
		void Move( int dir )
		{
			int& c = cursors[realism ? 1 : 0];
			do
			{
				c = (c + ((dir >= 0) ? 1 : (COUNT - 1))) % COUNT;
			} while (!Visible( c ));
		}

		/**
		 * Moves to another group of the list: forward, to the first line of the next group (circular); backward, to the first line of the
		 * group, or, if already there, of the previous group.
		 */
		void MoveGroup( int dir )
		{
			int& c = cursors[realism ? 1 : 0];
			const int g = ITEMS[c].group;
			if (dir > 0)
			{
				int i = c;
				do
				{
					i = (i + 1) % COUNT;
				} while ((!Visible( i ) || (ITEMS[i].group == g)) && (i != c));
				c = i;
				return;
			}
			int first = c;
			while ((first > 0) && (ITEMS[first - 1].group == g)) first--;
			if (first != c)
			{
				c = first;
				return;
			}
			int prev = (first + COUNT - 1) % COUNT;
			while (!Visible( prev )) prev = (prev + COUNT - 1) % COUNT;
			const int pg = ITEMS[prev].group;
			int f = prev;
			while ((f > 0) && (ITEMS[f - 1].group == pg)) f--;
			c = f;
		}

		/** Changes the value of the selected line one step. @return true if the value is different */
		bool Change( int dir )
		{
			const int c = Cursor();
			const double v = Step( c, values[c], dir );
			if (v == values[c]) return false;
			values[c] = v;
			dirty = true;
			return true;
		}

		/** The selected line goes back to its default. @return true if the value is different */
		bool ResetItem( void )
		{
			const int c = Cursor();
			if (values[c] == ITEMS[c].def) return false;
			values[c] = ITEMS[c].def;
			dirty = true;
			return true;
		}

		/** All the lines of the list that is shown go back to their defaults. @return true if any value is different */
		bool ResetAll( void )
		{
			bool changed = false;
			for (int i = 0; i < COUNT; i++)
			{
				if (Visible( i ) && (values[i] != ITEMS[i].def))
				{
					values[i] = ITEMS[i].def;
					changed = true;
				}
			}
			if (changed) dirty = true;
			return changed;
		}

		/** Position (in the list that is shown) of the first line to show so that the selected line is visible, near the middle, in a window of rows. */
		int FirstRow( int rows ) const
		{
			const int n = Lines();
			if (rows >= n) return 0;
			int pos = 0;
			for (int i = 0; i < Cursor(); i++) pos += Visible( i ) ? 1 : 0;
			int first = pos - (rows / 2);
			if (first < 0) first = 0;
			if (first > (n - rows)) first = n - rows;
			return first;
		}

		/** Index in the table of the option at a position of the list that is shown, -1 if there is none. */
		int IndexAt( int pos ) const
		{
			if (pos < 0) return -1;
			for (int i = 0; i < COUNT; i++)
			{
				if (Visible( i ) && (pos-- == 0)) return i;
			}
			return -1;
		}

	private:
		static int FirstOf( bool r )
		{
			for (int i = 0; i < COUNT; i++)
			{
				if (IsRealismGroup( ITEMS[i].group ) == r) return i;
			}
			return 0;
		}

		double values[COUNT];
		int cursors[2];
		bool realism;
		bool dirty;
	};
}


#endif// _SETTINGSMODEL_H_
