/****************************************************************************
  This file is part of Space Shuttle Vessel

  Water Spray Boiler subsystem definition


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
/******* SSV File Modification Notice *******
Date         Developer
2020/04/01   GLS
2020/05/10   GLS
2020/06/20   GLS
2021/06/06   GLS
2021/07/03   GLS
2021/08/23   GLS
2021/08/24   GLS
2022/09/29   GLS
********************************************/
/****************************************************************************
  This file is part of Space Shuttle Ultra

  Water Spray Boiler simulation definition



  Space Shuttle Ultra is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  Space Shuttle Ultra is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Space Shuttle Ultra; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

  See https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html or
  file Doc\Space Shuttle Ultra\GPL.txt for more details.

  **************************************************************************/
#ifndef _WSB_H_
#define _WSB_H_


#include "AtlantisSubsystem.h"
#include <discsignals.h>
#include "HydModel.h"


using namespace discsignals;

/**
 * Water spray boiler of an APU/hydraulic system: it cools the hydraulic fluid and the APU lube oil (model in HydModel.h).
 *
 * Bundle WSBn (n = 1 to 3), 16 lines:
 *   0 ready to start (out), 1 BOILER PWR, 2 controller A, 3 controller B, 4 N2 SUPPLY (in, from the panel R2)
 *   5 speed of the APU, 5 V = 103% (in, from the APU), 6 main pump at nominal pressure (in, from the APU)
 *   7 lube oil temperature, 8 hydraulic fluid temperature (out, 100 F per volt)
 *   9 water quantity, 10 hydraulic reservoir quantity (out, 5 V = 100%)
 *   11 the hydraulic system holds the pressure, 12 the bearings of the APU seized (out)
 *   13 hydraulic fluid hot (HYD RSVR T), 14 lube oil hot (APU TEMP), 15 the fluid goes through the boiler (out)
 * Failures (the settings MFD and the scenario): controller A or B, ice in the spray valves, nitrogen leak, hydraulic leak.
 */
class WSB : public AtlantisSubsystem
{
public:
	enum Failure {FAIL_CTRL_A = 0, FAIL_CTRL_B, FAIL_ICE_HYD, FAIL_ICE_OIL, FAIL_N2_LEAK, FAIL_HYD_LEAK, FAILURE_COUNT};

	WSB(AtlantisSubsystemDirector* _director, const string& _ident, unsigned short _ID);
	virtual ~WSB();

	void Realize() override;

	void OnPreStep(double simt, double simdt, double mjd) override;
	void OnPostStep(double simt, double simdt, double mjd) override;
	void OnPropagate(double simt, double simdt, double mjd) override;

	void OnSaveState(FILEHANDLE scn) const override;
	bool OnParseLine(const char* keyword, const char* line) override;

	double GetHydraulicFluidTemperature( void ) const {return th.hyd;}// [F]
	double GetLubeOilTemperature( void ) const {return th.oil;}// [F]
	double GetWater( void ) const {return th.water;}// [lb]
	double GetHydraulicQuantity( void ) const {return th.hydQty;}// [0, 1]
	bool BearingsSeized( void ) const {return th.seized;}

	/** @return true if the failure is active */
	bool GetFailure( int failure ) const;
	/** Sets or clears a failure (a leak is a constant rate). */
	void SetFailure( int failure, bool active );

	static const char* FailureName( int failure );

private:
	unsigned short usID;// from 1 to 3

	hydmodel::State th;
	bool thermalLoaded;// the temperatures came from the scenario
	bool hydCautionPrev, oilCautionPrev, seizedPrev;

	// indicates controller selected and if controller is powered
	DiscInPort Power, ControllerA, ControllerB;
	DiscInPort BoilerN2Supply;
	DiscOutPort Ready;
	DiscInPort ApuSpeed, PumpNorm;
	DiscOutPort OilTemp, HydTemp, WaterQty, HydQty, HydOk, LubeSeize, HydCaution, OilCaution, Through;
};

#endif// _WSB_H_
