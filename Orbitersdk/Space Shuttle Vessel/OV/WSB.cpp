/******* SSV File Modification Notice *******
Date         Developer
2020/06/20   GLS
2021/08/23   GLS
2021/08/24   GLS
2022/08/05   GLS
********************************************/
#include "WSB.h"
#include "Atlantis.h"
#include "SSVOptions.h"


namespace
{
	constexpr double APU_FULL_SPEED = 103.0;// [%] the speed of the line 5 of the bundle is relative to it
	constexpr double MAX_SUBSTEP = 5.0;// [s] the model is advanced in steps not longer than this (time acceleration)
	constexpr double RUNNING_OIL = 200.0;// [F] the temperatures that a system has when the scenario starts with the APU running
	constexpr double RUNNING_HYD = 150.0;
	constexpr double LEAK_N2 = 1.0 / 900.0;// [1/s] leak of nitrogen of the failure (the tank is empty in 15 minutes)
	constexpr double LEAK_HYD = 1.0 / 900.0;// [1/s] leak of the reservoir of the failure (empty in 15 minutes)

	const char* const FAILURE_NAMES[WSB::FAILURE_COUNT] = {"FailCtrlA", "FailCtrlB", "IceHyd", "IceOil", "N2Leak", "HydLeak"};
}


WSB::WSB(AtlantisSubsystemDirector *_director, const std::string &_ident, unsigned short _ID)
	: AtlantisSubsystem(_director, _ident), usID(_ID), th(hydmodel::InitialState()), thermalLoaded(false), hydCautionPrev(false), oilCautionPrev(false), seizedPrev(false)
{
}

WSB::~WSB()
{
}

const char* WSB::FailureName( int failure )
{
	return ((failure >= 0) && (failure < FAILURE_COUNT)) ? FAILURE_NAMES[failure] : "";
}

bool WSB::GetFailure( int failure ) const
{
	switch (failure)
	{
		case FAIL_CTRL_A: return th.ctrlFail[0];
		case FAIL_CTRL_B: return th.ctrlFail[1];
		case FAIL_ICE_HYD: return th.iceHyd;
		case FAIL_ICE_OIL: return th.iceOil;
		case FAIL_N2_LEAK: return th.n2Leak > 0.0;
		case FAIL_HYD_LEAK: return th.hydLeak > 0.0;
	}
	return false;
}

void WSB::SetFailure( int failure, bool active )
{
	switch (failure)
	{
		case FAIL_CTRL_A: th.ctrlFail[0] = active; break;
		case FAIL_CTRL_B: th.ctrlFail[1] = active; break;
		case FAIL_ICE_HYD: th.iceHyd = active; break;
		case FAIL_ICE_OIL: th.iceOil = active; break;
		case FAIL_N2_LEAK: th.n2Leak = active ? LEAK_N2 : 0.0; break;
		case FAIL_HYD_LEAK: th.hydLeak = active ? LEAK_HYD : 0.0; break;
		default: return;
	}
	if (active) oapiWriteLogV( "(SSV_OV) [INFO] Failure %s of the water spray boiler %d", FailureName( failure ), (int)usID );
}

void WSB::Realize()
{
	DiscreteBundle* pBundle=BundleManager()->CreateBundle(GetIdentifier(), 16);
	Ready.Connect(pBundle, 0);
	Power.Connect(pBundle, 1);
	ControllerA.Connect(pBundle, 2);
	ControllerB.Connect(pBundle, 3);
	BoilerN2Supply.Connect(pBundle, 4);
	ApuSpeed.Connect( pBundle, 5 );
	PumpNorm.Connect( pBundle, 6 );
	OilTemp.Connect( pBundle, 7 );
	HydTemp.Connect( pBundle, 8 );
	WaterQty.Connect( pBundle, 9 );
	HydQty.Connect( pBundle, 10 );
	HydOk.Connect( pBundle, 11 );
	LubeSeize.Connect( pBundle, 12 );
	HydCaution.Connect( pBundle, 13 );
	OilCaution.Connect( pBundle, 14 );
	Through.Connect( pBundle, 15 );
}

void WSB::OnPreStep(double simt, double simdt, double mjd)
{
	// indicate if WSB is ready for APU start
	if(Power && BoilerN2Supply && (ControllerA || ControllerB)) {
		Ready.SetLine();
	}
	else {
		Ready.ResetLine();
	}

	hydmodel::Input in;
	in.speed = (ApuSpeed.GetVoltage() / 5.0) * (APU_FULL_SPEED / 100.0);// of 100%
	if (in.speed > 1.2) in.speed = 1.2;
	in.pumpNorm = PumpNorm.IsSet();
	in.ambient = hydmodel::AMBIENT;
	in.powered = Power.IsSet();
	in.controller = ControllerA.IsSet() ? 1 : (ControllerB.IsSet() ? 2 : 0);
	in.n2Open = BoilerN2Supply.IsSet();
	in.load = hydmodel::LoadFromFlight( STS()->GetDynPressure(), STS()->GetThrusterGroupLevel( THGROUP_MAIN ) > 0.05 );

	if (STS()->GetOptions()->HydraulicThermal())
	{
		// the scenario started with the APU running and has no temperatures: the system is warm
		if (!thermalLoaded)
		{
			thermalLoaded = true;
			if (in.speed > 0.5)
			{
				th.oil = RUNNING_OIL;
				th.hyd = RUNNING_HYD;
			}
		}

		double left = simdt;
		while (left > 0.0)
		{
			const double dt = (left > MAX_SUBSTEP) ? MAX_SUBSTEP : left;
			hydmodel::Step( th, in, dt );
			left -= dt;
		}

		// the news of the system
		const bool hydCaution = hydmodel::HydCaution( th );
		const bool oilCaution = hydmodel::OilCaution( th );
		if (hydCaution && !hydCautionPrev) oapiWriteLogV( "(SSV_OV) [INFO] HYD %d RSVR T: hydraulic fluid at %.0f F", (int)usID, th.hyd );
		if (oilCaution && !oilCautionPrev) oapiWriteLogV( "(SSV_OV) [INFO] APU %d TEMP: lube oil at %.0f F", (int)usID, th.oil );
		if (th.seized && !seizedPrev) oapiWriteLogV( "(SSV_OV) [INFO] APU %d: the bearings seized (lube oil at %.0f F)", (int)usID, th.oil );
		hydCautionPrev = hydCaution;
		oilCautionPrev = oilCaution;
		seizedPrev = th.seized;
	}

	// outputs
	OilTemp.SetLine( static_cast<float>(hydmodel::TemperatureToVolts( th.oil )) );
	HydTemp.SetLine( static_cast<float>(hydmodel::TemperatureToVolts( th.hyd )) );
	WaterQty.SetLine( static_cast<float>(5.0 * hydmodel::Clamp01( th.water / hydmodel::WATER_TANK )) );
	HydQty.SetLine( static_cast<float>(5.0 * hydmodel::Clamp01( th.hydQty )) );
	if (hydmodel::HydraulicsOk( th )) HydOk.SetLine();
	else HydOk.ResetLine();
	if (th.seized) LubeSeize.SetLine();
	else LubeSeize.ResetLine();
	if (hydmodel::HydCaution( th )) HydCaution.SetLine();
	else HydCaution.ResetLine();
	if (hydmodel::OilCaution( th )) OilCaution.SetLine();
	else OilCaution.ResetLine();
	if (th.through) Through.SetLine();
	else Through.ResetLine();
}

void WSB::OnPostStep(double simt, double simdt, double mjd)
{
}

void WSB::OnPropagate(double simt, double simdt, double mjd)
{
}

void WSB::OnSaveState(FILEHANDLE scn) const
{
	oapiWriteScenario_float( scn, "Hyd", th.hyd );
	oapiWriteScenario_float( scn, "Oil", th.oil );
	oapiWriteScenario_float( scn, "Water", th.water );
	oapiWriteScenario_float( scn, "N2", th.n2 );
	oapiWriteScenario_int( scn, "Through", th.through ? 1 : 0 );
	oapiWriteScenario_float( scn, "HydDamage", th.hydDamage );
	oapiWriteScenario_float( scn, "OilOver", th.oilOver );
	oapiWriteScenario_float( scn, "HydQty", th.hydQty );
	oapiWriteScenario_int( scn, "Seized", th.seized ? 1 : 0 );
	for (int i = 0; i < FAILURE_COUNT; i++)
	{
		if (GetFailure( i )) oapiWriteScenario_int( scn, const_cast<char*>(FAILURE_NAMES[i]), 1 );
	}
}

bool WSB::OnParseLine(const char* keyword, const char* line)
{
	double v = 0.0;
	if (sscanf_s( line, "%lf", &v ) == 1)
	{
		if (!_stricmp( keyword, "Hyd" )) {th.hyd = v; thermalLoaded = true; return true;}
		if (!_stricmp( keyword, "Oil" )) {th.oil = v; thermalLoaded = true; return true;}
		if (!_stricmp( keyword, "Water" )) {th.water = hydmodel::Clamp01( v / hydmodel::WATER_TANK ) * hydmodel::WATER_TANK; return true;}
		if (!_stricmp( keyword, "N2" )) {th.n2 = hydmodel::Clamp01( v ); return true;}
		if (!_stricmp( keyword, "Through" )) {th.through = (v != 0.0); return true;}
		if (!_stricmp( keyword, "HydDamage" )) {th.hydDamage = hydmodel::Clamp01( v ); return true;}
		if (!_stricmp( keyword, "OilOver" )) {th.oilOver = (v < 0.0) ? 0.0 : v; return true;}
		if (!_stricmp( keyword, "HydQty" )) {th.hydQty = hydmodel::Clamp01( v ); return true;}
		if (!_stricmp( keyword, "Seized" )) {th.seized = (v != 0.0); return true;}
		for (int i = 0; i < FAILURE_COUNT; i++)
		{
			if (!_stricmp( keyword, FAILURE_NAMES[i] ))
			{
				SetFailure( i, v != 0.0 );
				return true;
			}
		}
	}
	return AtlantisSubsystem::OnParseLine(keyword, line);
}
