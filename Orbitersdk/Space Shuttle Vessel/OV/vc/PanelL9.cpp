/******* SSV File Modification Notice *******
Date         Developer
2020/06/20   GLS
2020/06/23   GLS
2021/08/24   GLS
2022/03/24   GLS
2022/03/26   GLS
2022/04/20   GLS
2022/08/05   GLS
2022/09/29   GLS
2024/12/30   GLS
********************************************/
#include "PanelL9.h"
#include "../Atlantis.h"
#include "../ParameterValues.h"
#include "vc_defs.h"
#include "StandardSwitch.h"
#include "DimmerKnob.h"
#include "../CabinLighting.h"
#include "../meshres_vc_l9.h"


namespace vc
{
	PanelL9::PanelL9( Atlantis* _sts ):AtlantisPanel( _sts, "L9" )
	{
		DefineMesh( MESHNAME_PANELL9 );

		// PS lighting (the switch has OFF at the top)
		Add( pPSFlood = new StdSwitch2( _sts, "PS FLOOD" ) );
		pPSFlood->SetLabel( 0, "ON" );
		pPSFlood->SetLabel( 1, "OFF" );
		pPSFlood->SetInitialPosition( 1 );
		Add( pPSFloodDim = new DimmerKnob( _sts, "PS FLOOD DIM-BRT", 0.5 ) );
	}

	PanelL9::~PanelL9()
	{
	}

	void PanelL9::DefineVC()
	{
		const VECTOR3 switch_rot = _V( 0.0, 0.0, -1.0 );

		AddAIDToMouseEventList( AID_L9 );

		pPSFlood->SetInitialAnimState( 0.5f );
		pPSFlood->DefineGroup( GRP_S11_L9_VC );
		pPSFlood->SetReference( _V( -1.6338, 2.4955, 12.3069 ), switch_rot );
		pPSFlood->SetMouseRegion( AID_L9, 0.126902f, 0.303009f, 0.169636f, 0.462959f );

		pPSFloodDim->DefineGroup( GRP_R3_L9_VC );
		pPSFloodDim->SetReference( _V( -1.6322, 2.4919, 12.3666 ), _V( -0.5446, -0.8387, 0.0 ) );
		pPSFloodDim->SetAngleRange( 270.0 );
		pPSFloodDim->SetMouseRegion( AID_L9, 0.216456f, 0.275821f, 0.284830f, 0.496442f );

		return;
	}

	void PanelL9::RegisterVC()
	{
		AtlantisPanel::RegisterVC();

		VECTOR3 ofs = STS()->GetOrbiterCoGOffset() + VC_OFFSET;

		oapiVCRegisterArea( AID_L9, PANEL_REDRAW_NEVER, PANEL_MOUSE_ONREPLAY | PANEL_MOUSE_LBDOWN | PANEL_MOUSE_LBUP | PANEL_MOUSE_LBPRESSED );
		oapiVCSetAreaClickmode_Quadrilateral( AID_L9,
			_V( -1.6965, 2.53376, 12.2204 ) + ofs, _V( -1.66009, 2.51012, 12.8038 ) + ofs,
			_V( -1.54444, 2.43502, 12.2204 ) + ofs, _V( -1.5438, 2.4346, 12.8038 ) + ofs );
		return;
	}

	void PanelL9::Realize()
	{
		DiscreteBundle* pLtgSw = STS()->BundleManager()->CreateBundle( "CABIN_LTG_SW", CABIN_LTG_SW_BUNDLE_LINES );
		pPSFlood->ConnectPort( 0, pLtgSw, CabinLighting::SW_PS_ON );
		DiscreteBundle* pLtgKnob = STS()->BundleManager()->CreateBundle( "CABIN_LTG_KNOB", CABIN_LTG_KNOB_BUNDLE_LINES );
		pPSFloodDim->Connect( pLtgKnob, CabinLighting::KNOB_PS );

		AtlantisPanel::Realize();
		return;
	}
}
