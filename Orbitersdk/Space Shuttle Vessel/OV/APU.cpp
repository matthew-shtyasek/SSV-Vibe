/******* SSV File Modification Notice *******
Date         Developer
2020/05/07   GLS
2020/06/20   GLS
2020/07/08   GLS
2021/08/23   GLS
2021/08/24   GLS
2022/01/10   GLS
2022/02/16   GLS
2022/07/24   GLS
2022/08/05   GLS
2022/09/29   GLS
2022/10/09   GLS
2022/10/21   GLS
2024/12/30   GLS
********************************************/
#include "APU.h"
#include "WSB.h"
#include "HydActuator.h"
#include "Atlantis.h"
#include "SSVOptions.h"
#include "../SSVSound.h"


APUInjectorWater::APUInjectorWater(AtlantisSubsystemDirector *_director, const std::string &_ident)
	: AtlantisSubsystem(_director, _ident), water(apumodel::WATER_FLOW_TIME)
{
}

APUInjectorWater::~APUInjectorWater()
{
}

void APUInjectorWater::Realize()
{
	DiscreteBundle* pBundle = BundleManager()->CreateBundle( "APU_INJ_WATER", 16 );
	for (int i = 0; i < 3; i++)
	{
		cmd[i].Connect( pBundle, i );
		ok[i].Connect( pBundle, 3 + i );
	}
	return;
}

void APUInjectorWater::OnPreStep(double simt, double simdt, double mjd)
{
	int open = 0;
	for (int i = 0; i < 3; i++)
	{
		if (cmd[i].IsSet() && (water > 0.0)) open++;
	}
	water = apumodel::WaterAfter( water, open, simdt );

	for (int i = 0; i < 3; i++)
	{
		if (water > 0.0) ok[i].SetLine();
		else ok[i].ResetLine();
	}
	return;
}

void APUInjectorWater::OnSaveState(FILEHANDLE scn) const
{
	oapiWriteScenario_float( scn, "Water", water );
	return;
}

bool APUInjectorWater::OnParseLine(const char* line)
{
	if (!_strnicmp( line, "Water", 5 ))
	{
		sscanf_s( line + 5, "%lf", &water );
		if (water < 0.0) water = 0.0;
		else if (water > apumodel::WATER_FLOW_TIME) water = apumodel::WATER_FLOW_TIME;
		return true;
	}
	return false;
}


APU::APU(AtlantisSubsystemDirector *_director, const std::string &_ident, int _ID)
	: AtlantisSubsystem(_director, _ident), ID(_ID)
{
	State=OFF;
	FuelMass[0]=FuelMass[1] = APU_FUEL_TANK_MASS;
	HydraulicPressure[0]=HydraulicPressure[1]=0.0;
	APUSpeed[0]=APUSpeed[1]=0.0;
	FuelPress[0]=FuelPress[1]=0.0;

	timeSinceStart = 0.0;
	autoShutdownLatch = false;
	overspeedShutdown = false;
	failed = false;
	failure = 0;
	thermalLoaded = false;
	coolingTime = 0.0;

	FuTkPress = Sensor( 0.0, 500.0 );
	InjTemp = Sensor( 0.0, 1000.0 );
	PumpTemp = Sensor( 0.0, 1000.0 );
	ValveTemp = Sensor( 0.0, 1000.0 );
	OilTemp = Sensor( 0.0, 500.0 );

	HYD_MN_PMP_P[0] = Sensor( 0.0, 4000.0 );
	HYD_MN_PMP_P[1] = Sensor( 0.0, 4000.0 );
	HYD_MN_PMP_P[2] = Sensor( 0.0, 4000.0 );
}

APU::~APU()
{
}

double APU::GetHydraulicPressure() const
{
	return HydraulicPressure[0];
}

double APU::GetFuelLevel() const
{
	return FuelMass[0];
}

double APU::GetAPUSpeed() const
{
	return APUSpeed[0];
}

double APU::GetSubsystemMass() const
{
	return FuelMass[0] * LBM2KG;
}

void APU::Realize()
{
	std::string name="R2_To_"+GetIdentifier();
	DiscreteBundle *pBundle=BundleManager()->CreateBundle(name, 16);

	APU_Run.Connect(pBundle, 0);
	APU_CntlrPwr.Connect(pBundle, 1);
	APU_FuelTankValves.Connect(pBundle, 2);
	APU_HydPumpPress.Connect(pBundle, 3);
	APU_HydraulicPress.Connect(pBundle, 4);
	APU_ReadyToStart.Connect(pBundle, 5);
	APU_InjectorCool.Connect(pBundle, 6);
	APU_SpeedHigh.Connect(pBundle, 7);
	APU_AutoShutdownEnable.Connect(pBundle, 8);
	APU_UnderspeedLight.Connect(pBundle, 9);
	APU_OverspeedLight.Connect(pBundle, 10);

	char cbuf[255];
	sprintf_s(cbuf, 255, "WSB%d", ID);
	pBundle=BundleManager()->CreateBundle(cbuf, 16);
	WSB_Ready.Connect(pBundle, 0);
	WSB_Speed.Connect( pBundle, 5 );
	WSB_PumpNorm.Connect( pBundle, 6 );
	WSB_OilTemp.Connect( pBundle, 7 );
	WSB_HydOk.Connect( pBundle, 11 );
	WSB_Seize.Connect( pBundle, 12 );

	sprintf_s( cbuf, 255, "APU_%d_SENSORS", ID );
	pBundle = BundleManager()->CreateBundle( cbuf, 16 );
	FuTkPress.Connect( pBundle, 0 );
	InjTemp.Connect( pBundle, 1 );
	PumpTemp.Connect( pBundle, 2 );
	ValveTemp.Connect( pBundle, 3 );
	OilTemp.Connect( pBundle, 4 );

	pBundle = BundleManager()->CreateBundle( "APU_INJ_WATER", 16 );
	InjectorWaterCmd.Connect( pBundle, ID - 1 );
	InjectorWaterOk.Connect( pBundle, 3 + ID - 1 );

	pBundle = BundleManager()->CreateBundle( "HYD_PMP_PRESS", 16 );
	HYD_MN_PMP_P[0].Connect( pBundle, (ID - 1) * 3 );
	HYD_MN_PMP_P[1].Connect( pBundle, ((ID - 1) * 3) + 1 );
	HYD_MN_PMP_P[2].Connect( pBundle, ((ID - 1) * 3) + 2 );
	return;
}

/** Normal random number (mean 0, sigma 1). */
static double Gauss( void )
{
	double s = 0.0;
	for (int i = 0; i < 12; i++) s += oapiRand();
	return s - 6.0;
}

namespace
{
	// where the gas of an APU leaves the aft compartment: the access doors 50-1/50-2 and the aft vent doors (frame of the mesh, from the groups DOOR_50-x and VENT_DOOR_x_8 of OV.msh).
	// The drawing of the equipment of the aft compartment (NASA CR-193229) puts APU 1 and 2 on the port side and APU 3 on the starboard side; the exact places inside are not known.
	const VECTOR3 FX_POS[3] = {_V( -2.80, -0.65, -10.30 ), _V( -2.96, -1.56, -12.07 ), _V( 2.80, -0.65, -10.30 )};
	const VECTOR3 FX_DIR[3] = {_V( -0.55, 0.45, -0.70 ), _V( -0.55, 0.30, -0.78 ), _V( 0.55, 0.45, -0.70 )};
}

void APU::StartFire( const char* reason )
{
	if (burning) return;
	burning = true;
	fireOut = 0.0;
	oapiWriteLogV( "(SSV_OV) [WARNING] APU %d: fire in the aft compartment (%s)", ID, reason );
}

void APU::Fail( const char* reason, FailMode mode, bool collateral )
{
	failed = true;
	State = OFF;
	APUSpeed[1] = 0.0;
	FuelPress[1] = 0.0;
	HydraulicPressure[1] = 0.0;
	oapiWriteLogV( "(SSV_OV) [WARNING] APU %d is destroyed (%s): %s", ID, (mode == FAIL_EXPLOSION) ? "explosion" : ((mode == FAIL_FIRE) ? "fire" : "seized"), reason );
	if (mode != FAIL_SEIZED)
	{
		// the fuel lines are broken
		leaking = true;
		leakFactor = apumodel::LeakFactor( Gauss() );
		leakTime = 0.0;
		leakFireChecked = false;

		// the fire of a hydrazine leak needs air
		const bool air = apumodel::AirForFire( STS()->GetAtmPressure() );
		if (air && ((mode == FAIL_FIRE) || (oapiRand() < apumodel::EXPLOSION_IGNITES))) StartFire( "the hydrazine ignited" );

		// the pump is a part of the APU and the lines are next to it: the hydraulic system can lose its fluid
		const double lossChance = (mode == FAIL_EXPLOSION) ? apumodel::HYD_LOSS_EXPLOSION : apumodel::HYD_LOSS_FIRE;
		WSB* wsb = STS()->GetWSB( ID );
		if ((wsb != NULL) && (oapiRand() < lossChance))
		{
			wsb->SetFailure( WSB::FAIL_HYD_LEAK, true );
			oapiWriteLogV( "(SSV_OV) [WARNING] APU %d: the hydraulic system %d loses its fluid", ID, ID );
		}
	}
	if (mode == FAIL_EXPLOSION)
	{
		boomTime = apumodel::BOOM_TIME;
		debrisTime = apumodel::DEBRIS_TIME;
		if (!oapiCameraInternal()) SoundPlay( STS()->GetSound(), APU_1_BANG + ID - 1 );// muffled bang, heard outside only (not from the cockpit)
		if (collateral && (oapiRand() < apumodel::COLLATERAL_EXPLOSION)) SpreadDamage( true );
	}
	return;
}

void APU::UpdateFire( double simt, double simdt, bool leakNow )
{
	const double p = STS()->GetAtmPressure();
	if (!failed)
	{
		// random fire (option, to train): a working APU leaks and burns in the air, like the injector stem of STS-9
		const double rate = STS()->GetOptions()->APURandomFireRate();
		if ((rate > 0.0) && (State == ON) && (oapiRand() < apumodel::RandomFireProbability( simdt, rate, p)))
			Fail( "random failure: a crack of the injector stem, the hydrazine leaks and burns", FAIL_FIRE );
		return;
	}
	if (!burning)
	{
		if (leakNow && (oapiRand() < apumodel::IgnitionProbability( simdt, p ))) StartFire( "the leaking hydrazine ignited in the air" );
		return;
	}
	if (!apumodel::AirForFire( p ))
	{
		burning = false;
		oapiWriteLogV( "(SSV_OV) [INFO] APU %d: the fire went out (no air)", ID );
	}
	else if (!leakNow)
	{
		fireOut += simdt;
		if (fireOut >= apumodel::FIRE_AFTER_LEAK)
		{
			burning = false;
			oapiWriteLogV( "(SSV_OV) [INFO] APU %d: the fire burned out", ID );
		}
	}
	else fireOut = 0.0;
	return;
}

void APU::CreateEffects( void )
{
	if (psFlash != NULL) return;
	const VECTOR3 ofs = STS()->GetOrbiterCoGOffset();// local frame = frame of the mesh + ofs
	const VECTOR3 pos = FX_POS[ID - 1] + ofs;
	VECTOR3 dir = FX_DIR[ID - 1];
	dir = dir / length( dir );
	static SURFHANDLE texFlame = oapiRegisterParticleTexture( "SSV\\OV\\SSMEstream" );
	static SURFHANDLE texSmoke = oapiRegisterParticleTexture( "contrail1" );
	static PARTICLESTREAMSPEC flash = {0, 0.8, 80, 8, 0.6, 0.7, 10, 3, PARTICLESTREAMSPEC::EMISSIVE, PARTICLESTREAMSPEC::LVL_FLAT, 1, 1, PARTICLESTREAMSPEC::ATM_FLAT, 1, 1, 0};
	static PARTICLESTREAMSPEC fire = {0, 0.35, 40, 4, 0.3, 1.0, 3, 4, PARTICLESTREAMSPEC::EMISSIVE, PARTICLESTREAMSPEC::LVL_FLAT, 1, 1, PARTICLESTREAMSPEC::ATM_FLAT, 1, 1, 0};
	static PARTICLESTREAMSPEC smoke = {0, 0.6, 20, 5, 0.35, 10, 2.5, 4, PARTICLESTREAMSPEC::DIFFUSE, PARTICLESTREAMSPEC::LVL_FLAT, 1, 1, PARTICLESTREAMSPEC::ATM_FLAT, 1, 1, 0};
	static PARTICLESTREAMSPEC vapor = {0, 0.15, 12, 3, 0.4, 4, 0.8, 2, PARTICLESTREAMSPEC::DIFFUSE, PARTICLESTREAMSPEC::LVL_FLAT, 1, 1, PARTICLESTREAMSPEC::ATM_FLAT, 1, 1, 0};
	flash.tex = texFlame;
	fire.tex = texFlame;
	smoke.tex = texSmoke;
	vapor.tex = texSmoke;
	psFlash = STS()->AddParticleStream( &flash, pos, dir, &lvFlash );
	psFire = STS()->AddParticleStream( &fire, pos, dir, &lvFire );
	psSmoke = STS()->AddParticleStream( &smoke, pos, dir, &lvSmoke );
	psVapor = STS()->AddParticleStream( &vapor, pos, dir, &lvVapor );
	COLOUR4 diff = {1.0f, 0.55f, 0.2f, 0.0f}, zero = {0.0f, 0.0f, 0.0f, 0.0f}, amb = {0.3f, 0.15f, 0.05f, 0.0f};
	fxLight = STS()->AddPointLight( pos, 40.0, 1.0, 0.05, 0.002, diff, zero, amb );
	if (fxLight != NULL) fxLight->Activate( false );
}

void APU::UpdateEffects( double simdt, bool leakNow )
{
	const bool anything = (boomTime > 0.0) || (debrisTime > 0.0) || burning || leakNow;
	if (!anything)
	{
		if (psFlash != NULL)
		{
			lvFlash = lvFire = lvSmoke = lvVapor = 0.0;
			if (fxLight != NULL) fxLight->Activate( false );
		}
		return;
	}
	CreateEffects();
	if (boomTime > 0.0) boomTime = (std::max)( 0.0, boomTime - simdt );
	if (debrisTime > 0.0) debrisTime = (std::max)( 0.0, debrisTime - simdt );
	lvFlash = (boomTime > 0.0) ? 1.0 : 0.0;
	lvFire = burning ? 1.0 : 0.0;
	lvSmoke = (burning || (debrisTime > 0.0)) ? 1.0 : 0.0;
	lvVapor = (leakNow && !burning) ? 1.0 : 0.0;
	if (fxLight != NULL)
	{
		const bool lit = (boomTime > 0.0) || burning;
		fxLight->Activate( lit );
		if (lit) fxLight->SetIntensity( (boomTime > 0.0) ? 1.0 : (0.4 + (0.6 * oapiRand())) );// the fire flickers
	}
	return;
}

void APU::SpreadDamage( bool explosion )
{
	APU* others[3];
	int n = 0;
	for (int i = 1; i <= 3; i++)
	{
		APU* a = STS()->GetAPU( i );
		if ((a != NULL) && (a != this) && !a->failed) others[n++] = a;
	}
	if (n == 0) return;
	int k = static_cast<int>(oapiRand() * n);
	if (k >= n) k = n - 1;
	char reason[160];
	sprintf_s( reason, sizeof( reason ), "%s of APU %d damaged it", explosion ? "the explosion" : "the fire of the hydrazine leak", ID );
	// the damage of an explosion does not spread again by explosion; a leak that it starts can still burn another APU
	others[k]->Fail( reason, explosion ? FAIL_EXPLOSION : FAIL_FIRE, false );
	return;
}

void APU::OnPreStep(double simt, double simdt, double mjd)
{
	// the hydrazine leaks from the broken lines while the fuel tank isolation valves are open; a leak that lasts can burn another APU
	if (failed && leaking)
	{
		if (APU_FuelTankValves.IsSet() && (FuelMass[1] > 0.0))
		{
			if ((simt - leakJitterTime) >= 1.0)
			{
				leakJitterTime = simt;
				leakJitter = Gauss();
			}
			FuelMass[1] = (std::max)( 0.0, FuelMass[1] - (apumodel::LeakRate( leakFactor, leakJitter ) * simdt) );
			leakTime += simdt;
			if (!leakFireChecked && (leakTime >= apumodel::FIRE_SPREAD_TIME))
			{
				leakFireChecked = true;
				oapiWriteLogV( "(SSV_OV) [WARNING] APU %d: the hydrazine leaks for %.0f s (FUEL TK VLV not closed), the fire can spread to another APU", ID, leakTime );
				if (oapiRand() < apumodel::COLLATERAL_FIRE) SpreadDamage( false );
			}
		}
		else if (FuelMass[1] <= 0.0) leaking = false;
	}

	// fire and effects seen from outside
	{
		const bool leakNow = failed && leaking && APU_FuelTankValves.IsSet() && (FuelMass[1] > 0.0);
		UpdateFire( simt, simdt, leakNow );
		UpdateEffects( simdt, leakNow );
	}

	// the bearings seize when the lube oil is not cooled
	if (WSB_Seize.IsSet() && !failed) Fail( "the lube oil was too hot and the bearings seized", FAIL_SEIZED );

	// the APUs of a scenario that starts with the APU running are hot
	if (!thermalLoaded)
	{
		thermalLoaded = true;
		if ((State == START) || (State == ON)) th = apumodel::RunningThermal();
	}

	const bool run = APU_Run.IsSet();// APU OPERATE in START/RUN
	const bool injectorCoolSwitch = APU_InjectorCool.IsSet();// APU OPERATE in INJECTOR COOL
	const bool autoEnabled = APU_AutoShutdownEnable.IsSet();
	const bool high = APU_SpeedHigh.IsSet();

	// the auto shutdown of the controller is reset with the APU OPERATE switch in OFF. If the FUEL TK VLV switch is still in OPEN, the
	// fuel tank isolation valves open again and the fuel reaches the gas generator, that can be hot
	if (autoShutdownLatch && !run && !injectorCoolSwitch && (State != START) && (State != ON))
	{
		autoShutdownLatch = false;
		if (APU_FuelTankValves.IsSet() && !failed)
		{
			const double p = apumodel::DetonationProbability( th );
			oapiWriteLogV( "(SSV_OV) [WARNING] APU %d: the fuel tank isolation valves opened again after the auto shutdown (the FUEL TK VLV switch was not in CLOSE)", ID );
			if (oapiRand() < p) Fail( "hydrazine detonation, the fuel reached a gas generator above the temperature limits", FAIL_EXPLOSION );
		}
	}
	const bool valvesOpen = APU_FuelTankValves.IsSet() && !autoShutdownLatch;

	const bool running = (State == START) || (State == ON);

	// injector cooling with water (APU OPERATE in INJECTOR COOL, only with the APU not running)
	const bool coolRequest = injectorCoolSwitch && !run && !running && !failed;
	if (coolRequest) InjectorWaterCmd.SetLine();
	else InjectorWaterCmd.ResetLine();
	const bool cooling = coolRequest && InjectorWaterOk.IsSet();
	if (cooling)
	{
		coolingTime += simdt;
		if ((coolingTime - simdt) < 225.0 && coolingTime >= 225.0) oapiWriteLogV( "(SSV_OV) [INFO] APU %d cooldown: 225 s of injector cooling", ID );
	}
	else coolingTime = 0.0;
	apumodel::ThermalStep( th, running, cooling, simdt );

	APU_UnderspeedLight.ResetLine();
	APU_OverspeedLight.ResetLine();

	switch(State) {
		case SHUTDOWN:
			if (SoundIsPlaying( STS()->GetSound(), APU_1_SHUTDOWN + ID - 1 ))
			{
				SoundStop( STS()->GetSound(), APU_1_START + ID - 1 );
				SoundStop( STS()->GetSound(), APU_1_RUNNING + ID - 1 );
				SoundPlay( STS()->GetSound(), APU_1_SHUTDOWN + ID - 1 );
			}
		case OFF:
			if(APUSpeed[1]>5) APUSpeed[1]=max(APUSpeed[1]-15.0*simdt, 0.0);
			else {
				APUSpeed[1]=0.0;
				State=OFF;
			}
			if(FuelPress[1]>25) FuelPress[1] -= 200*simdt;
			else FuelPress[1] = 0.0;
			if(HydraulicPressure[1]>50) HydraulicPressure[1] -= 200*simdt;
			else HydraulicPressure[1] = 0.0;

			if(ReadyToStart()) {
				APU_ReadyToStart.SetLine();
				if(run && (State == OFF)) {
					// restarting an APU that has not cooled down or that had an overspeed can destroy it
					double p = apumodel::DetonationProbability( th );
					if (overspeedShutdown) p = 1.0;
					if (oapiRand() < p)
					{
						Fail( overspeedShutdown ? "uncontained overspeed, the APU was started after an overspeed shutdown" : "hydrazine detonation, the APU was started above the temperature limits", FAIL_EXPLOSION );
					}
					else
					{
						State=START;
						timeSinceStart = 0.0;
						SoundPlay( STS()->GetSound(), APU_1_START + ID - 1 );
					}
				}
			}
			else APU_ReadyToStart.ResetLine();

			break;
		case START:
		case ON:
			// sound only plays in external view, so keep calling PlayVesselWave to make sure sound plays if we switch to external view
			if(State==START && SoundIsPlaying( STS()->GetSound(), APU_1_START + ID - 1 )) SoundPlay( STS()->GetSound(), APU_1_START + ID - 1 );
			else SoundPlay( STS()->GetSound(), APU_1_RUNNING + ID - 1, true );

			timeSinceStart += simdt;

			// the controller without power or without fuel to the gas generator: shutdown
			if ((!APU_CntlrPwr.IsSet()) || (!valvesOpen)) State=SHUTDOWN;
			if(!run) {
				State=SHUTDOWN;
				SoundPlay( STS()->GetSound(), APU_1_SHUTDOWN + ID - 1 );
			}

			if(APU_HydPumpPress.IsSet())
				FuelMass[1]-=APU_FUEL_TANK_FLOWRATE[0]*simdt;
			else FuelMass[1]-=APU_FUEL_TANK_FLOWRATE[1]*simdt;
			if (FuelMass[1] < 0) FuelMass[1] = 0;

			if(State==START) {
				const double target = apumodel::TargetSpeed( high );
				if(APUSpeed[1]>=95.0) {
					APUSpeed[1]=target+(oapiRand()-0.5);
					State=ON;
				}
				else {
					APUSpeed[1]+=15.0*simdt;
				}

				APU_ReadyToStart.SetLine(); //in START sequence, RTS talkback is still gray
			}
			else if (State == ON) {
				// the speed that the controller holds; without fuel the speed falls, and the failures of the scenario (valves that fail) move it
				double target = apumodel::TargetSpeed( high );
				if (FuelMass[1] <= 0.0) target = 0.0;
				if (failure == 1) target = 70.0;
				else if (failure == 2) target = 135.0;
				const double dv = target - APUSpeed[1];
				const double maxstep = 6.0 * simdt;
				APUSpeed[1] += max( -maxstep, min( maxstep, dv ) );
				if (fabs( dv ) < 1.0) APUSpeed[1] += 0.75*(oapiRand()-0.5);

				APU_ReadyToStart.ResetLine(); //if APU is on, RTS talkback displays barberpole
			}

			// underspeed and overspeed: the lights of panel F7 and, with the APU AUTO SHUT DOWN switch in ENABLE, the shutdown that closes
			// the fuel tank isolation valves and the secondary fuel control valve
			{
				const bool under = apumodel::UnderspeedTrip( APUSpeed[1], timeSinceStart );
				const bool over = apumodel::OverspeedTrip( APUSpeed[1] );
				if (under) APU_UnderspeedLight.SetLine();
				if (over) APU_OverspeedLight.SetLine();
				if (autoEnabled && (under || over) && ((State == START) || (State == ON)))
				{
					autoShutdownLatch = true;
					if (over) overspeedShutdown = true;
					State = SHUTDOWN;
					oapiWriteLogV( "(SSV_OV) [INFO] APU %d auto shutdown (%s)", ID, over ? "overspeed" : "underspeed" );
					SoundPlay( STS()->GetSound(), APU_1_SHUTDOWN + ID - 1 );
				}
			}

			if(FuelPress[1]<1400) FuelPress[1]+=200*simdt;
			else if(FuelPress[1]>1500) FuelPress[1]-=200*simdt;
			else if(FuelPress[1]<1425 || FuelPress[1]>1480)
				FuelPress[1]=1455 - 10.0 * oapiRand();
			else FuelPress[1] += (2.0 * oapiRand() - 1.0);

			if (!WSB_HydOk.IsSet())
			{
				// the reservoir is empty (leak of the hydraulic system): the pump can not hold the pressure
				if (HydraulicPressure[1] > 400.0 * simdt) HydraulicPressure[1] -= 400.0 * simdt;
				else HydraulicPressure[1] = 0.0;
			}
			else {
				// set points of the pump of this vehicle: 3000 psi (the regulation is 2900-3100) in NORM, about 800 psi (500-1000) in LOW,
				// with the tolerances of the part and the slow variation (noise) of the options of the realism page
				const disp::Mode dm = STS()->GetHydraulicDispersion();
				const uint64_t seed = STS()->GetToleranceSeed();
				double hiSet = 3000.0;
				double loSet = 800.0;
				if (dm.tolerances)
				{
					hiSet = disp::Abs( hiSet, hydact::PUMP_SIGMA, disp::Fixed( seed, disp::id::HYD_PUMP_SET + ID - 1 ), dm.scale );
					loSet = disp::Abs( loSet, hydact::PUMP_LOW_SIGMA, disp::Fixed( seed, disp::id::HYD_PUMP_LOW + ID - 1 ), dm.scale );
				}
				if (!dm.noise) pumpWander = 0.0;
				else if ((simt - pumpWanderTime) >= 1.0)
				{
					pumpWanderTime = simt;
					pumpWander = disp::Abs( 0.0, hydact::PUMP_NOISE_SIGMA, pumpNoise.Normal(), dm.scale );
				}
				hiSet += pumpWander;
				loSet += 0.3 * pumpWander;
				if (hiSet < 2900.0) hiSet = 2900.0;
				if (hiSet > 3100.0) hiSet = 3100.0;
				if (loSet < 500.0) loSet = 500.0;
				if (loSet > 1000.0) loSet = 1000.0;

				if(APU_HydPumpPress.IsSet()) {
					if(HydraulicPressure[1]<(hiSet - 50.0)) {
						if(HydraulicPressure[1]>(hiSet - 100.0))
							HydraulicPressure[1]=hiSet-(10*oapiRand());
						else
							HydraulicPressure[1]+=200*simdt;
					}
					else if(HydraulicPressure[1]>(hiSet + 50.0))
						HydraulicPressure[1]-=200*simdt;
					else
						HydraulicPressure[1]+=2.0 * oapiRand() - 1.0;
				}
				else {
					if(HydraulicPressure[1]<(loSet - 50.0)) {
						if(HydraulicPressure[1]>(loSet - 100.0))
							HydraulicPressure[1]=loSet-10*oapiRand();
						else HydraulicPressure[1]+=200*simdt;
					}
					else if(HydraulicPressure[1]>(loSet + 50.0))
						HydraulicPressure[1]-=200*simdt;
					else {
						HydraulicPressure[1]+=(2.0*oapiRand() - 1.0);
					}
				}
			}

			break;
	}
}

void APU::OnPropagate(double simt, double simdt, double mjd)
{
	FuelMass[0]=FuelMass[1];
	FuelPress[0]=FuelPress[1];
	HydraulicPressure[0]=HydraulicPressure[1];
	APUSpeed[0]=APUSpeed[1];
}

void APU::OnPostStep(double simt, double simdt, double mjd)
{
	// flight data log: when the switches or the state change, and periodically while the APU works or is hot
	{
		const char* const states[] = {"OFF", "START", "ON", "SHUTDOWN"};
		const int key[10] = {static_cast<int>(State), APU_Run.IsSet() ? 1 : 0, APU_InjectorCool.IsSet() ? 1 : 0, APU_FuelTankValves.IsSet() ? 1 : 0, APU_CntlrPwr.IsSet() ? 1 : 0,
			APU_HydPumpPress.IsSet() ? 1 : 0, ReadyToStart() ? 1 : 0, failed ? 1 : 0, autoShutdownLatch ? 1 : 0, IsLeaking() ? 1 : 0};
		const bool changed = memcmp( key, hydLogKey, sizeof( key ) ) != 0;
		const bool hot = (State != OFF) || (th.injector > apumodel::LIMIT_INJECTOR_F) || (th.pump > apumodel::LIMIT_PUMP_F) || (th.valve > apumodel::LIMIT_VALVE_F);
		const double acc = oapiGetTimeAcceleration();
		if (changed || (hot && ((simt - hydLogTime) >= max( 10.0, acc ))))
		{
			flightlog::Writer* log = STS()->HydLog();
			if (log != NULL)
			{
				const int s = static_cast<int>(State);
				log->Line( "APU id=%d simt=%.1f acc=%.1f why=%s state=%s run=%d inj_cool=%d tk_valves=%d cntlr_pwr=%d pump_norm=%d ready=%d latch=%d failed=%d wsb_ready=%d speed=%.1f hyd_psi=%.0f fuel_lb=%.1f inj_F=%.0f pump_F=%.0f valve_F=%.0f detonation_p=%.2f leak=%d",
					ID, simt, acc, changed ? "change" : "periodic", ((s >= 0) && (s < 4)) ? states[s] : "?", key[1], key[2], key[3], key[4], key[5], key[6], key[8], key[7], WSB_Ready.IsSet() ? 1 : 0,
					APUSpeed[0], HydraulicPressure[0], FuelMass[0], th.injector, th.pump, th.valve, apumodel::DetonationProbability( th ), key[9] );
			}
			memcpy( hydLogKey, key, sizeof( key ) );
			hydLogTime = simt;
		}
	}

	FuTkPress.SetValue( (FuelMass[0] / APU_FUEL_TANK_MASS) * 365 );
	APU_HydraulicPress.SetLine((float)(HydraulicPressure[0]/1000.0));

	InjTemp.SetValue( th.injector );
	PumpTemp.SetValue( th.pump );
	ValveTemp.SetValue( th.valve );
	OilTemp.SetValue( WSB_OilTemp.GetVoltage() * 100.0 );

	// for the heat of the hydraulic fluid and of the lube oil of the boiler
	WSB_Speed.SetLine( static_cast<float>((APUSpeed[0] >= 103.0) ? 5.0 : (5.0 * APUSpeed[0] / 103.0)) );
	if (APU_HydPumpPress.IsSet()) WSB_PumpNorm.SetLine();
	else WSB_PumpNorm.ResetLine();

	HYD_MN_PMP_P[0].SetValue( HydraulicPressure[0] );
	HYD_MN_PMP_P[1].SetValue( HydraulicPressure[0] );
	HYD_MN_PMP_P[2].SetValue( HydraulicPressure[0] );
}

void APU::OnSaveState(FILEHANDLE scn) const
{
	oapiWriteScenario_int( scn, "State", State );
	oapiWriteScenario_float( scn, "FuelMass", FuelMass[0] );
	oapiWriteScenario_float( scn, "FuelPress", FuelPress[0] );
	oapiWriteScenario_float( scn, "HydPress", HydraulicPressure[0] );
	oapiWriteScenario_float( scn, "Speed", APUSpeed[0] );
	oapiWriteScenario_float( scn, "Injector", th.injector );
	oapiWriteScenario_float( scn, "Pump", th.pump );
	oapiWriteScenario_float( scn, "Valve", th.valve );
	oapiWriteScenario_float( scn, "Soak", th.soak );
	if (autoShutdownLatch) oapiWriteScenario_int( scn, "Latch", 1 );
	if (leaking)
	{
		char leak[96];
		sprintf_s( leak, sizeof( leak ), "%.3f %.1f %d", leakFactor, leakTime, leakFireChecked ? 1 : 0 );
		oapiWriteScenario_string( scn, "Leak", leak );
	}
	if (overspeedShutdown) oapiWriteScenario_int( scn, "Overspeed", 1 );
	if (failed) oapiWriteScenario_int( scn, "Failed", 1 );
	if (failure != 0) oapiWriteScenario_int( scn, "Failure", failure );
}

bool APU::OnParseLine(const char *line)
{
	if (!_strnicmp( line, "State", 5 ))
	{
		sscanf_s( line + 5, "%d", &State );
		if ((State == START) || (State == ON)) timeSinceStart = 1000.0;
		return true;
	}
	else if (!_strnicmp( line, "FuelMass", 8 ))
	{
		sscanf_s( line + 8, "%lf", &FuelMass[0] );
		if (FuelMass[0] > APU_FUEL_TANK_MASS) FuelMass[0] = APU_FUEL_TANK_MASS;
		else if (FuelMass[0] < 0.0) FuelMass[0] = 0.0;
		FuelMass[1]=FuelMass[0];
		return true;
	}
	else if (!_strnicmp( line, "FuelPress", 9 ))
	{
		sscanf_s( line + 9, "%lf", &FuelPress[0] );
		FuelPress[1]=FuelPress[0];
		return true;
	}
	else if (!_strnicmp( line, "HydPress", 8 ))
	{
		sscanf_s( line + 8, "%lf", &HydraulicPressure[0] );
		HydraulicPressure[1]=HydraulicPressure[0];
		return true;
	}
	else if (!_strnicmp( line, "Speed", 5 ))
	{
		sscanf_s( line + 5, "%lf", &APUSpeed[0] );
		// old scenarios saved speeds that no running APU has (for example 305%); the controller would trip at once (overspeed or underspeed)
		if (((State == START) || (State == ON)) && ((APUSpeed[0] < apumodel::UNDERSPEED_PCT) || (APUSpeed[0] > apumodel::OVERSPEED_PCT)))
		{
			oapiWriteLogV( "(SSV_OV) [WARNING] APU %d: speed %.0f%% of the scenario is out of the range of a running APU, 100%% used", ID, APUSpeed[0] );
			APUSpeed[0] = 100.0;
		}
		APUSpeed[1]=APUSpeed[0];
		return true;
	}
	else if (!_strnicmp( line, "Injector", 8 ))
	{
		sscanf_s( line + 8, "%lf", &th.injector );
		thermalLoaded = true;
		return true;
	}
	else if (!_strnicmp( line, "Pump", 4 ))
	{
		sscanf_s( line + 4, "%lf", &th.pump );
		thermalLoaded = true;
		return true;
	}
	else if (!_strnicmp( line, "Valve", 5 ))
	{
		sscanf_s( line + 5, "%lf", &th.valve );
		thermalLoaded = true;
		return true;
	}
	else if (!_strnicmp( line, "Soak", 4 ))
	{
		sscanf_s( line + 4, "%lf", &th.soak );
		thermalLoaded = true;
		return true;
	}
	else if (!_strnicmp( line, "Latch", 5 ))
	{
		autoShutdownLatch = true;
		return true;
	}
	else if (!_strnicmp( line, "Leak", 4 ))
	{
		int checked = 0;
		sscanf_s( line + 4, "%lf %lf %d", &leakFactor, &leakTime, &checked );
		leakFireChecked = (checked != 0);
		leaking = true;
		return true;
	}
	else if (!_strnicmp( line, "Overspeed", 9 ))
	{
		overspeedShutdown = true;
		return true;
	}
	else if (!_strnicmp( line, "Failed", 6 ))
	{
		failed = true;
		return true;
	}
	else if (!_strnicmp( line, "Failure", 7 ))
	{
		sscanf_s( line + 7, "%d", &failure );
		return true;
	}
	return false;
}

bool APU::ReadyToStart() const
{
	return (!failed && FuelMass[0]>0.0 && APU_CntlrPwr.IsSet() && APU_FuelTankValves.IsSet() && !autoShutdownLatch && !APU_HydPumpPress.IsSet() && WSB_Ready.IsSet());
}
