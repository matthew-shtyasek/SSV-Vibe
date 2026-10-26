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
	enum Kind {KIND_BOOL = 0, KIND_INT, KIND_FLOAT};

	enum Group {GROUP_FLIGHT = 0, GROUP_RMS, GROUP_CABIN, GROUP_SYSTEMS, GROUP_COUNT};

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
	};

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
		RMS_PAYLOAD_MASS_EFFECT,
		RMS_REACTION,
		RMS_CENTER_OF_MASS,
		RMS_INERTIA,
		CABIN_LIGHTING,
		VC_FREE_MOVE,
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
		{"RMSPayloadMassEffect", "Payload slows the arm", "The rates of the arm decrease with the mass of the payload that it holds", KIND_BOOL, 0.0, 0.0, 1.0, 1.0, false, GROUP_RMS},
		{"RMSReaction", "Arm pushes the orbiter", "Reaction forces of the arm and of its payload on the orbiter (it turns with the DAP in FREE)", KIND_BOOL, 0.0, 0.0, 1.0, 1.0, false, GROUP_RMS},
		{"RMSCenterOfMass", "C.g. follows the arm", "The center of mass of the orbiter follows the arm and the payload that it holds", KIND_BOOL, 0.0, 0.0, 1.0, 1.0, false, GROUP_RMS},
		{"RMSInertia", "Inertia with the arm", "The moments of inertia of the orbiter change with the arm and the payload that it holds", KIND_BOOL, 0.0, 0.0, 1.0, 1.0, false, GROUP_RMS},
		{"CabinLighting", "Cabin lighting", "Floodlights and panel lighting follow the switches and knobs (needs the local lights of Orbiter)", KIND_BOOL, 1.0, 0.0, 1.0, 1.0, false, GROUP_CABIN},
		{"VCFreeMove", "Free head movement", "Ctrl+M: arrows and Ins/Del move the head around the station", KIND_BOOL, 1.0, 0.0, 1.0, 1.0, false, GROUP_CABIN},
	};
	static_assert( (sizeof( ITEMS ) / sizeof( ITEMS[0] )) == COUNT, "the table of the options and its indices" );

	inline const char* GroupName( int group )
	{
		switch (group)
		{
			case GROUP_FLIGHT: return "FLIGHT";
			case GROUP_RMS: return "RMS";
			case GROUP_CABIN: return "CABIN";
			case GROUP_SYSTEMS: return "SYSTEMS";
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
	 * State of the list of the settings MFD: the values, the line that is selected and what changed. The values of the editor are applied by
	 * the MFD to the options of the vessel.
	 */
	class Editor
	{
	public:
		Editor( void )
		{
			cursor = 0;
			dirty = false;
			for (int i = 0; i < COUNT; i++) values[i] = ITEMS[i].def;
		}

		void SetValues( const double* v )
		{
			for (int i = 0; i < COUNT; i++) values[i] = Clamp( i, v[i] );
			dirty = false;
		}

		double Value( int i ) const { return values[i]; }
		int Cursor( void ) const { return cursor; }

		/** True after a change that was not written to the file. */
		bool IsDirty( void ) const { return dirty; }
		void MarkSaved( void ) { dirty = false; }

		/** Moves the selected line (the list is circular). */
		void Move( int dir )
		{
			cursor = (cursor + ((dir >= 0) ? 1 : (COUNT - 1))) % COUNT;
		}

		/**
		 * Moves to another group of the list: forward, to the first line of the next group (circular); backward, to the first line of the
		 * group, or, if already there, of the previous group.
		 */
		void MoveGroup( int dir )
		{
			const int g = ITEMS[cursor].group;
			if (dir > 0)
			{
				int i = cursor;
				do
				{
					i = (i + 1) % COUNT;
				} while ((ITEMS[i].group == g) && (i != cursor));
				cursor = i;
				return;
			}
			int first = cursor;
			while ((first > 0) && (ITEMS[first - 1].group == g)) first--;
			if (first != cursor)
			{
				cursor = first;
				return;
			}
			const int prev = (first + COUNT - 1) % COUNT;
			const int pg = ITEMS[prev].group;
			int f = prev;
			while ((f > 0) && (ITEMS[f - 1].group == pg)) f--;
			cursor = f;
		}

		/** Changes the value of the selected line one step. @return true if the value is different */
		bool Change( int dir )
		{
			const double v = Step( cursor, values[cursor], dir );
			if (v == values[cursor]) return false;
			values[cursor] = v;
			dirty = true;
			return true;
		}

		/** The selected line goes back to its default. @return true if the value is different */
		bool ResetItem( void )
		{
			if (values[cursor] == ITEMS[cursor].def) return false;
			values[cursor] = ITEMS[cursor].def;
			dirty = true;
			return true;
		}

		/** All the lines go back to their defaults. @return true if any value is different */
		bool ResetAll( void )
		{
			bool changed = false;
			for (int i = 0; i < COUNT; i++)
			{
				if (values[i] != ITEMS[i].def)
				{
					values[i] = ITEMS[i].def;
					changed = true;
				}
			}
			if (changed) dirty = true;
			return changed;
		}

		/** Index of the first line to show so that the selected line is visible, near the middle, in a window of rows. */
		int FirstRow( int rows ) const
		{
			if (rows >= COUNT) return 0;
			int first = cursor - (rows / 2);
			if (first < 0) first = 0;
			if (first > (COUNT - rows)) first = COUNT - rows;
			return first;
		}

	private:
		double values[COUNT];
		int cursor;
		bool dirty;
	};
}


#endif// _SETTINGSMODEL_H_
