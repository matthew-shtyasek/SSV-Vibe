/****************************************************************************
  This file is part of Space Shuttle Vessel

  Cabin lighting (Crew Operations Manual 2.15): the floodlights of the flight deck and the lighting of the panels

  The switches and the DIM/BRT knobs of the panels O6, O8, R10, L9 and A6U write on three discrete bundles, and this subsystem lights the
  cabin with them:
    CABIN_LTG_SW    (switches)  0 CDR glareshield BRIGHT, 1 CDR glareshield VAR, 2 PLT glareshield BRIGHT, 3 PLT glareshield VAR,
                                4 CDR SEAT, 5 CDR CTR CNSL, 6 PLT SEAT, 7 PLT CTR CNSL, 8 MS flood ON, 9 PS flood ON, 10 OS flood ON,
                                11 EMERGENCY LIGHTING ON (switch of the panel C3)
    CABIN_LTG_KNOB  (knobs, 5 V at BRT)  0 CDR glareshield, 1 PLT glareshield, 2 CDR seat/console, 3 PLT seat/console, 4 MS flood,
                                5 PS flood, 6 OS flood
    CABIN_LTG_PANEL (knobs, 5 V at BRT)  0 PANEL LEFT/CENTER, 1 PANEL LEFT OVHD, 2 PANEL RIGHT/CENTER, 3 PANEL RIGHT OVHD, 4 PANEL MS, 5 PANEL OS

  Emergency lighting: with the switch EMERGENCY LIGHTING of C3 on, the fixtures of the emergency system (left glareshield, orbit station and
  one of the middeck floodlights) are at full bright. The panel ML18F with the second switch is not in the virtual cockpit.
  The middeck has two floodlights (they stand for the eight of the ceiling), controlled by the option MiddeckLights: the panel MO13Q with
  its switches is not in the virtual cockpit either.

  The floodlights are light emitters of Orbiter in the cockpit views (they need the local lights of Orbiter), and the panel lighting scales
  the self illumination of the panel meshes. Option CabinLighting of the vessel configuration switches it all off (the panels as they are
  in the meshes).


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
#ifndef _CABINLIGHTING_H_
#define _CABINLIGHTING_H_


#include "AtlantisSubsystem.h"
#include "CabinLightingData.h"
#include <DiscInPort.h>
#include <OrbiterAPI.h>


using namespace discsignals;


inline constexpr int CABIN_LTG_SW_BUNDLE_LINES = 16;
inline constexpr int CABIN_LTG_KNOB_BUNDLE_LINES = 16;
inline constexpr int CABIN_LTG_PANEL_BUNDLE_LINES = 16;


class CabinLighting: public AtlantisSubsystem
{
public:
	/** Lines of the bundle CABIN_LTG_SW */
	enum SwitchLine {SW_CDR_GLARE_BRIGHT = 0, SW_CDR_GLARE_VAR, SW_PLT_GLARE_BRIGHT, SW_PLT_GLARE_VAR, SW_CDR_SEAT, SW_CDR_CONSOLE, SW_PLT_SEAT, SW_PLT_CONSOLE, SW_MS_ON, SW_PS_ON, SW_OS_ON, SW_EMERGENCY, SW_COUNT};
	/** Lines of the bundle CABIN_LTG_KNOB */
	enum KnobLine {KNOB_CDR_GLARE = 0, KNOB_PLT_GLARE, KNOB_CDR_SEAT, KNOB_PLT_SEAT, KNOB_MS, KNOB_PS, KNOB_OS, KNOB_COUNT};
	/** The floodlights */
	enum Flood {FLOOD_CDR_GLARE = 0, FLOOD_PLT_GLARE, FLOOD_CDR_SEAT, FLOOD_PLT_SEAT, FLOOD_MS, FLOOD_PS, FLOOD_OS, FLOOD_MID_FWD, FLOOD_MID_AFT, FLOOD_COUNT};

	CabinLighting( AtlantisSubsystemDirector* _director, const string& _ident );
	~CabinLighting();

	bool SingleParamParseLine( void ) const override {return true;};
	void Realize( void ) override;
	void OnPostStep( double simt, double simdt, double mjd ) override;
	void ShiftCG( const VECTOR3& shift ) override;

	/** @return the level of a floodlight [0, 1] */
	double GetFloodLevel( int flood ) const;

private:
	void CreateLights( void );
	void PlaceLights( void );

	LightEmitter* lamp[FLOOD_COUNT];
	double level[FLOOD_COUNT];
	int seatMode[2];// 0 = off, 1 = seat, 2 = console (CDR, PLT)
	bool lightsCreated;

	DiscInPort sw[SW_COUNT];
	DiscInPort knob[KNOB_COUNT];
	DiscInPort panelKnob[cabinlight::PNL_COUNT - 1];
};


#endif// _CABINLIGHTING_H_
