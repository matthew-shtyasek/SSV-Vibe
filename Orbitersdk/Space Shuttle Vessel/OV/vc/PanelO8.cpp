/******* SSV File Modification Notice *******
Date         Developer
2020/05/10   GLS
2020/06/20   GLS
2020/06/23   GLS
2021/06/02   GLS
2021/08/24   GLS
2022/03/24   GLS
2022/03/26   GLS
2022/04/20   GLS
2022/05/29   GLS
2022/08/05   GLS
2022/09/29   GLS
2024/12/30   GLS
********************************************/
#include "PanelO8.h"
#include "StandardSwitch.h"
#include "DimmerKnob.h"
#include "../CabinLighting.h"
#include "../Atlantis.h"
#include "../ParameterValues.h"
#include "vc_defs.h"
#include "../meshres_vc_o8.h"


namespace vc
{
	PanelO8::PanelO8( Atlantis* psts ):AtlantisPanel( psts, "O8" )
	{
		DefineMesh( MESHNAME_PANELO8 );

		Add( pRadarAltimeter[0] = new StdSwitch2( psts, "RADAR ALTIMETER 1" ) );
		pRadarAltimeter[0]->SetLabel( 0, "OFF" );
		pRadarAltimeter[0]->SetLabel( 1, "ON" );

		Add( pRadarAltimeter[1] = new StdSwitch2( psts, "RADAR ALTIMETER 2" ) );
		pRadarAltimeter[1]->SetLabel( 0, "OFF" );
		pRadarAltimeter[1]->SetLabel( 1, "ON" );

		Add( pAnnunciatorLampTest = new StdSwitch3( psts, "ANNUNCIATOR LAMP TEST" ) );

		// cabin lighting
		Add( pRGlareshieldFlood = new StdSwitch3( psts, "R GLARESHIELD FLOOD" ) );
		pRGlareshieldFlood->SetLabel( 0, "OFF" );
		pRGlareshieldFlood->SetLabel( 1, "VAR" );
		pRGlareshieldFlood->SetLabel( 2, "BRIGHT" );
		pRGlareshieldFlood->SetInitialPosition( 0 );
		Add( pRGlareshieldDim = new DimmerKnob( psts, "R GLARESHIELD FLOOD DIM-BRT", 0.5 ) );
		Add( pRSeatConsoleFlood = new StdSwitch3( psts, "RIGHT SEAT-CTR CNSL FLOOD" ) );
		pRSeatConsoleFlood->SetLabel( 0, "CTR CNSL" );
		pRSeatConsoleFlood->SetLabel( 1, "OFF" );
		pRSeatConsoleFlood->SetLabel( 2, "SEAT" );
		pRSeatConsoleFlood->SetInitialPosition( 1 );
		Add( pRSeatConsoleDim = new DimmerKnob( psts, "RIGHT SEAT-CTR CNSL FLOOD DIM-BRT", 0.5 ) );
		Add( pPanelRightCenter = new DimmerKnob( psts, "PANEL RIGHT-CENTER OFF-BRT", 1.0 ) );
		Add( pPanelRightOverhead = new DimmerKnob( psts, "PANEL RIGHT OVERHEAD OFF-BRT", 1.0 ) );
	}

	PanelO8::~PanelO8()
	{
	}

	void PanelO8::DefineVC()
	{
		VECTOR3 switch_rot = _V( 1, 0, 0 );

		AddAIDToMouseEventList( AID_O8 );

		pRGlareshieldFlood->SetInitialAnimState( 0.5f );
		pRGlareshieldFlood->DefineGroup( GRP_S3_O8_VC );
		pRGlareshieldFlood->SetReference( _V( 0.5517, 3.1229, 13.8707 ), switch_rot );
		pRGlareshieldFlood->SetMouseRegion( AID_O8, 0.662334f, 0.351931f, 0.711505f, 0.391908f );

		pRGlareshieldDim->DefineGroup( GRP_R1_O8_VC );
		pRGlareshieldDim->SetReference( _V( 0.6251, 3.1229, 13.8669 ), _V( 0.0, -0.9718, -0.2357 ) );
		pRGlareshieldDim->SetAngleRange( 270.0 );
		pRGlareshieldDim->SetPointerOffset( 135.0 );
		pRGlareshieldDim->SetMouseRegion( AID_O8, 0.792045f, 0.339262f, 0.870719f, 0.394402f );

		pRSeatConsoleFlood->SetInitialAnimState( 0.5f );
		pRSeatConsoleFlood->DefineGroup( GRP_A1_S1_O8_VC );
		pRSeatConsoleFlood->SetReference( _V( 0.4167, 3.1222, 13.8731 ), switch_rot );
		pRSeatConsoleFlood->SetMouseRegion( AID_O8, 0.396791f, 0.355417f, 0.445962f, 0.395393f );

		pRSeatConsoleDim->DefineGroup( GRP_R6_O8_VC );
		pRSeatConsoleDim->SetReference( _V( 0.4719, 3.1217, 13.8731 ), _V( 0.0, -0.9718, -0.2357 ) );
		pRSeatConsoleDim->SetAngleRange( 270.0 );
		pRSeatConsoleDim->SetPointerOffset( 135.0 );
		pRSeatConsoleDim->SetMouseRegion( AID_O8, 0.490627f, 0.348051f, 0.569300f, 0.403190f );

		pPanelRightCenter->DefineGroup( GRP_R2_O8_VC );
		pPanelRightCenter->SetReference( _V( 0.5540, 3.1647, 13.6962 ), _V( 0.0, -0.9718, -0.2357 ) );
		pPanelRightCenter->SetAngleRange( 270.0 );
		pPanelRightCenter->SetPointerOffset( 135.0 );
		pPanelRightCenter->SetMouseRegion( AID_O8, 0.652222f, 0.097039f, 0.730896f, 0.152179f );

		pPanelRightOverhead->DefineGroup( GRP_R3_O8_VC );
		pPanelRightOverhead->SetReference( _V( 0.6235, 3.1647, 13.6962 ), _V( 0.0, -0.9718, -0.2357 ) );
		pPanelRightOverhead->SetAngleRange( 270.0 );
		pPanelRightOverhead->SetPointerOffset( 135.0 );
		pPanelRightOverhead->SetMouseRegion( AID_O8, 0.788918f, 0.097038f, 0.867591f, 0.152178f );


		pRadarAltimeter[0]->SetInitialAnimState( 0.5f );
		pRadarAltimeter[0]->DefineGroup( GRP_S4_O8_VC );
		pRadarAltimeter[0]->SetReference( _V( 0.2651, 3.1652, 13.6946 ), switch_rot );
		pRadarAltimeter[0]->SetMouseRegion( AID_O8, 0.089737f, 0.106703f, 0.135184f, 0.140805f );

		pRadarAltimeter[1]->SetInitialAnimState( 0.5f );
		pRadarAltimeter[1]->DefineGroup( GRP_S5_O8_VC );
		pRadarAltimeter[1]->SetReference( _V( 0.2651, 3.1652, 13.6946 ), switch_rot );
		pRadarAltimeter[1]->SetMouseRegion( AID_O8, 0.157729f, 0.106310f, 0.201778f, 0.140387f );

		pAnnunciatorLampTest->SetInitialAnimState( 0.5f );
		pAnnunciatorLampTest->DefineGroup( GRP_S18_O8_VC );
		pAnnunciatorLampTest->SetReference( _V( 0.6000, 3.0978, 13.9709 ), switch_rot );
		pAnnunciatorLampTest->SetMouseRegion( AID_O8, 0.775565f, 0.498687f, 0.814904f, 0.530986f );
		pAnnunciatorLampTest->SetSpringLoaded( true, 0 );
		pAnnunciatorLampTest->SetSpringLoaded( true, 2 );
	}

	void PanelO8::Realize()
	{
		DiscreteBundle* pLtgSw = STS()->BundleManager()->CreateBundle( "CABIN_LTG_SW", CABIN_LTG_SW_BUNDLE_LINES );
		pRGlareshieldFlood->ConnectPort( 2, pLtgSw, CabinLighting::SW_PLT_GLARE_BRIGHT );
		pRGlareshieldFlood->ConnectPort( 1, pLtgSw, CabinLighting::SW_PLT_GLARE_VAR );
		pRSeatConsoleFlood->ConnectPort( 2, pLtgSw, CabinLighting::SW_PLT_SEAT );
		pRSeatConsoleFlood->ConnectPort( 0, pLtgSw, CabinLighting::SW_PLT_CONSOLE );
		DiscreteBundle* pLtgKnob = STS()->BundleManager()->CreateBundle( "CABIN_LTG_KNOB", CABIN_LTG_KNOB_BUNDLE_LINES );
		pRGlareshieldDim->Connect( pLtgKnob, CabinLighting::KNOB_PLT_GLARE );
		pRSeatConsoleDim->Connect( pLtgKnob, CabinLighting::KNOB_PLT_SEAT );
		DiscreteBundle* pLtgPanel = STS()->BundleManager()->CreateBundle( "CABIN_LTG_PANEL", CABIN_LTG_PANEL_BUNDLE_LINES );
		pPanelRightCenter->Connect( pLtgPanel, cabinlight::PNL_RIGHT_CTR - 1 );
		pPanelRightOverhead->Connect( pLtgPanel, cabinlight::PNL_RIGHT_OVHD - 1 );

		discsignals::DiscreteBundle* pBundle = STS()->BundleManager()->CreateBundle( "RDR_ALT", 16 );
		pRadarAltimeter[0]->ConnectPort( 1, pBundle, 0 );
		pRadarAltimeter[1]->ConnectPort( 1, pBundle, 1 );

		pBundle = STS()->BundleManager()->CreateBundle( "ACA", 16 );
		pAnnunciatorLampTest->ConnectPort( 2, pBundle, 8 );
		pAnnunciatorLampTest->ConnectPort( 0, pBundle, 9 );

		AtlantisPanel::Realize();
	}

	void PanelO8::RegisterVC()
	{
		AtlantisPanel::RegisterVC();

		VECTOR3 ofs = STS()->GetOrbiterCoGOffset() + VC_OFFSET;

		oapiVCRegisterArea( AID_O8, PANEL_REDRAW_NEVER, PANEL_MOUSE_ONREPLAY | PANEL_MOUSE_LBDOWN | PANEL_MOUSE_LBUP | PANEL_MOUSE_LBPRESSED );
		oapiVCSetAreaClickmode_Quadrilateral( AID_O8,
			_V( 0.202423, 3.1858, 13.6083 ) + ofs, _V( 0.710853, 3.1858, 13.6083 ) + ofs,
			_V( 0.202423, 3.01485, 14.3133 ) + ofs, _V( 0.710853, 3.01474, 14.3133 ) + ofs );
	}

}
