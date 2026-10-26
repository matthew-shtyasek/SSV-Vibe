#include "CabinLighting.h"
#include "Atlantis.h"
#include "LightingMath.h"
#include "SSVOptions.h"
#include "ParameterValues.h"
#include "vc/AtlantisPanel.h"
#include <DiscreteBundleManager.h>


namespace
{
	// positions of the floodlights in the frame of the virtual cockpit mesh [m]
	const VECTOR3 POS_CDR_GLARE = _V( -0.70, 2.58, 14.35 );
	const VECTOR3 POS_PLT_GLARE = _V( 0.70, 2.58, 14.35 );
	const VECTOR3 POS_CDR_SEAT = _V( -0.50, 3.00, 13.80 );// above and ahead of the seat
	const VECTOR3 POS_PLT_SEAT = _V( 0.50, 3.00, 13.80 );
	const VECTOR3 POS_MS = _V( 1.00, 3.30, 12.90 );
	const VECTOR3 POS_PS = _V( -1.00, 3.30, 12.90 );
	const VECTOR3 POS_OS = _V( 0.00, 3.35, 12.70 );

	// the light of the console is moved toward the center console when CTR CNSL is selected
	const VECTOR3 CONSOLE_SHIFT_CDR = _V( 0.45, -0.25, 0.20 );
	const VECTOR3 CONSOLE_SHIFT_PLT = _V( -0.45, -0.25, 0.20 );

	constexpr double RAMP_RATE = 4.0;// [1/s] fluorescent lamps and the eye: the level does not jump

	// color of the light (fluorescent lamps are cold white, the incandescent lamps of the seat lights are warm)
	const COLOUR4 COL_FLUORESCENT = {0.95f, 1.0f, 1.0f, 0.0f};
	const COLOUR4 COL_INCANDESCENT = {1.0f, 0.82f, 0.58f, 0.0f};
	const COLOUR4 COL_SPECULAR = {0.0f, 0.0f, 0.0f, 0.0f};
	const COLOUR4 COL_AMBIENT = {0.0f, 0.0f, 0.0f, 0.0f};
}


CabinLighting::CabinLighting( AtlantisSubsystemDirector* _director, const string& _ident )
	: AtlantisSubsystem( _director, _ident ), lightsCreated( false )
{
	for (int i = 0; i < FLOOD_COUNT; i++)
	{
		lamp[i] = NULL;
		level[i] = 0.0;
	}
	seatMode[0] = seatMode[1] = lightmath::SC_OFF;
}

CabinLighting::~CabinLighting()
{
}

void CabinLighting::Realize( void )
{
	DiscreteBundle* pBundle = BundleManager()->CreateBundle( "CABIN_LTG_SW", CABIN_LTG_SW_BUNDLE_LINES );
	for (int i = 0; i < SW_COUNT; i++) sw[i].Connect( pBundle, i );

	pBundle = BundleManager()->CreateBundle( "CABIN_LTG_KNOB", CABIN_LTG_KNOB_BUNDLE_LINES );
	for (int i = 0; i < KNOB_COUNT; i++) knob[i].Connect( pBundle, i );

	pBundle = BundleManager()->CreateBundle( "CABIN_LTG_PANEL", CABIN_LTG_PANEL_BUNDLE_LINES );
	for (int i = 0; i < (cabinlight::PNL_COUNT - 1); i++) panelKnob[i].Connect( pBundle, i );

	CreateLights();
	return;
}

void CabinLighting::CreateLights( void )
{
	if (lightsCreated) return;
	lightsCreated = true;

	const VECTOR3 ofs = STS()->GetOrbiterCoGOffset() + VC_OFFSET;
	const VECTOR3* pos[FLOOD_COUNT] = {&POS_CDR_GLARE, &POS_PLT_GLARE, &POS_CDR_SEAT, &POS_PLT_SEAT, &POS_MS, &POS_PS, &POS_OS};
	for (int i = 0; i < FLOOD_COUNT; i++)
	{
		const bool incandescent = (i == FLOOD_CDR_SEAT) || (i == FLOOD_PLT_SEAT);
		const double range = incandescent ? 3.0 : 4.5;
		lamp[i] = STS()->AddPointLight( *pos[i] + ofs, range, 1.0, 0.0, incandescent ? 1.2 : 0.5, incandescent ? COL_INCANDESCENT : COL_FLUORESCENT, COL_SPECULAR, COL_AMBIENT );
		lamp[i]->SetVisibility( LightEmitter::VIS_COCKPIT );
		lamp[i]->SetIntensity( 0.0 );
		lamp[i]->Activate( false );
	}
	return;
}

void CabinLighting::PlaceLights( void )
{
	const VECTOR3 ofs = STS()->GetOrbiterCoGOffset() + VC_OFFSET;
	const VECTOR3* pos[FLOOD_COUNT] = {&POS_CDR_GLARE, &POS_PLT_GLARE, &POS_CDR_SEAT, &POS_PLT_SEAT, &POS_MS, &POS_PS, &POS_OS};
	for (int i = 0; i < FLOOD_COUNT; i++)
	{
		if (lamp[i] == NULL) continue;
		VECTOR3 p = *pos[i];
		if ((i == FLOOD_CDR_SEAT) && (seatMode[0] == lightmath::SC_CONSOLE)) p += CONSOLE_SHIFT_CDR;
		if ((i == FLOOD_PLT_SEAT) && (seatMode[1] == lightmath::SC_CONSOLE)) p += CONSOLE_SHIFT_PLT;
		lamp[i]->SetPosition( p + ofs );
	}
	return;
}

void CabinLighting::ShiftCG( const VECTOR3& shift )
{
	// the position of the lights is in the frame of the vessel, which moved
	PlaceLights();
	return;
}

double CabinLighting::GetFloodLevel( int flood ) const
{
	if ((flood < 0) || (flood >= FLOOD_COUNT)) return 0.0;
	return level[flood];
}

void CabinLighting::OnPostStep( double simt, double simdt, double mjd )
{
	const bool enabled = STS()->GetOptions()->CabinLighting();

	double target[FLOOD_COUNT] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	int mode[2] = {lightmath::SC_OFF, lightmath::SC_OFF};
	double panelScale[cabinlight::PNL_COUNT] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

	if (enabled)
	{
		target[FLOOD_CDR_GLARE] = lightmath::GlareshieldLevel( sw[SW_CDR_GLARE_BRIGHT].IsSet(), sw[SW_CDR_GLARE_VAR].IsSet(), lightmath::KnobFromVolts( knob[KNOB_CDR_GLARE].GetVoltage() ) );
		target[FLOOD_PLT_GLARE] = lightmath::GlareshieldLevel( sw[SW_PLT_GLARE_BRIGHT].IsSet(), sw[SW_PLT_GLARE_VAR].IsSet(), lightmath::KnobFromVolts( knob[KNOB_PLT_GLARE].GetVoltage() ) );

		mode[0] = lightmath::SeatConsoleSelect( sw[SW_CDR_SEAT].IsSet(), sw[SW_CDR_CONSOLE].IsSet() );
		mode[1] = lightmath::SeatConsoleSelect( sw[SW_PLT_SEAT].IsSet(), sw[SW_PLT_CONSOLE].IsSet() );
		target[FLOOD_CDR_SEAT] = (mode[0] != lightmath::SC_OFF) ? lightmath::KnobLevel( lightmath::KnobFromVolts( knob[KNOB_CDR_SEAT].GetVoltage() ) ) : 0.0;
		target[FLOOD_PLT_SEAT] = (mode[1] != lightmath::SC_OFF) ? lightmath::KnobLevel( lightmath::KnobFromVolts( knob[KNOB_PLT_SEAT].GetVoltage() ) ) : 0.0;

		target[FLOOD_MS] = lightmath::SwitchedLevel( sw[SW_MS_ON].IsSet(), lightmath::KnobFromVolts( knob[KNOB_MS].GetVoltage() ) );
		target[FLOOD_PS] = lightmath::SwitchedLevel( sw[SW_PS_ON].IsSet(), lightmath::KnobFromVolts( knob[KNOB_PS].GetVoltage() ) );
		target[FLOOD_OS] = lightmath::SwitchedLevel( sw[SW_OS_ON].IsSet(), lightmath::KnobFromVolts( knob[KNOB_OS].GetVoltage() ) );

		for (int i = 0; i < (cabinlight::PNL_COUNT - 1); i++) panelScale[i + 1] = lightmath::PanelScale( lightmath::KnobFromVolts( panelKnob[i].GetVoltage() ) );
	}

	// the lamps
	bool moved = false;
	for (int i = 0; i < FLOOD_COUNT; i++)
	{
		level[i] = lightmath::Ramp( level[i], target[i], RAMP_RATE, simdt );
		if (lamp[i] != NULL)
		{
			lamp[i]->SetIntensity( lightmath::EmitterIntensity( level[i], 1.0 ) );
			lamp[i]->Activate( level[i] > 0.001 );
		}
	}
	if ((mode[0] != seatMode[0]) || (mode[1] != seatMode[1]))
	{
		seatMode[0] = mode[0];
		seatMode[1] = mode[1];
		moved = true;
	}
	if (moved) PlaceLights();

	// the panels
	for (int i = 1; i < cabinlight::PNL_COUNT; i++) STS()->SetPanelLighting( i, panelScale[i] );
	return;
}
