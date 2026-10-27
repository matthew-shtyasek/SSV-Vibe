/****************************************************************************
  This file is part of Space Shuttle Vessel

  Dimmer knob VC Component (OFF/BRT, DIM/BRT)

  A rotary control with a stop at each end. The group of the knob in the mesh has the pointer straight up (the middle of the travel); the
  knob is animated from the first stop (angle/2 counterclockwise) to the second (angle/2 clockwise). The position is given on a line of
  a discrete bundle as a voltage (5 V at the second stop).


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
#ifndef __DIMMERKNOB_H
#define __DIMMERKNOB_H


#include "AtlantisVCComponent.h"
#include <DiscOutPort.h>
#include <vector>


namespace vc
{
	using namespace discsignals;

	class DimmerKnob:public AtlantisVCComponent
	{
		private:
			double pos;// [0:1]
			double ang;// travel [deg]
			double offset;// [deg] clockwise: the pointer of the mesh is this much counterclockwise from the middle of the travel

			DiscOutPort output;

			UINT grpIndex;
			int calGroup;// group of the calibration settings (0 none, 1 O6/O8, 2 A6U, 3 L9, 4 R10)
			double lastOff, lastTrav;
			bool lastRev;
			void GetParams( double& off, double& trav, bool& rev ) const;
			static std::vector<DimmerKnob*>& Registry( void );
			MGROUP_ROTATE* rotTravel;
			UINT anim_rot;

			void OnPositionChange( void );

		public:
			/**
			 * @param defaultPos position of the knob when the scenario does not set it [0:1]
			 */
			DimmerKnob( Atlantis* _sts, const std::string& _ident, double defaultPos );
			virtual ~DimmerKnob();

			bool OnParseLine( const char* line ) override;
			bool GetStateString( unsigned long ulBufferSize, char* pszBuffer ) override;
			void DefineGroup( UINT _grpIndex );
			void SetAngleRange( double ang );
			/** The pointer of the mesh at rest is this many degrees counterclockwise from the middle of the travel (12 o'clock) */
			void SetPointerOffset( double deg ) { offset = deg; }
			/** The pointer offset, travel and direction come from the settings KnobTop, KnobAft, KnobL9, KnobR10 (1 to 4), live. */
			void SetCalibrationGroup( int group ) { calGroup = group; }
			/** Applies the settings that changed to all the knobs. */
			static void RefreshAll( void );
			void Connect( DiscreteBundle* pBundle, unsigned short usLine );
			void DefineVCAnimations( UINT vc_idx ) override;
			void Realize( void ) override;
			bool OnMouseEvent( int _event, float x, float y ) override;

			/** @return the position of the knob [0:1] */
			double GetPosition( void ) const { return pos; }
	};
}

#endif// __DIMMERKNOB_H
