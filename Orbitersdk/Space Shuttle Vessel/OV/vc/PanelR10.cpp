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
#include "PanelR10.h"
#include "../Atlantis.h"
#include "../ParameterValues.h"
#include "vc_defs.h"
#include "StandardSwitch.h"
#include "DimmerKnob.h"
#include "../CabinLighting.h"
#include "../meshres_vc_r10.h"


namespace vc
{
	PanelR10::PanelR10( Atlantis* _sts ):AtlantisPanel( _sts, "R10" )
	{
		DefineMesh( MESHNAME_PANELR10 );

		// MS lighting
		Add( pMSFlood = new StdSwitch2( _sts, "MS LIGHTING FLOOD" ) );
		pMSFlood->SetLabel( 0, "OFF" );
		pMSFlood->SetLabel( 1, "ON" );
		pMSFlood->SetInitialPosition( 0 );
		Add( pMSFloodDim = new DimmerKnob( _sts, "MS LIGHTING FLOOD DIM-BRT", 0.5 ) );
		Add( pPanelMS = new DimmerKnob( _sts, "MS LIGHTING PANEL OFF-BRT", 1.0 ) );
	}

	PanelR10::~PanelR10()
	{
	}

	void PanelR10::DefineVC()
	{
		const VECTOR3 switch_rot = _V( 0.0, 0.0, -1.0 );

		AddAIDToMouseEventList( AID_R10 );

		pMSFlood->SetInitialAnimState( 0.5f );
		pMSFlood->DefineGroup( GRP_S12_R10_VC );
		pMSFlood->SetReference( _V( 1.6240, 2.4906, 12.3585 ), switch_rot );
		pMSFlood->SetMouseRegion( AID_R10, 0.790722f, 0.311514f, 0.836241f, 0.520646f );

		pMSFloodDim->DefineGroup( GRP_R3_R10_VC );
		pMSFloodDim->SetReference( _V( 1.6247, 2.4897, 12.3055 ), _V( 0.5446, -0.8387, 0.0 ) );
		pMSFloodDim->SetAngleRange( 300.0 );
		pMSFloodDim->SetCalibrationGroup( 4 );
		pMSFloodDim->SetMouseRegion( AID_R10, 0.873934f, 0.285688f, 0.946764f, 0.574146f );

		pPanelMS->DefineGroup( GRP_R4_R10_VC );
		pPanelMS->SetReference( _V( 1.6247, 2.4897, 12.4120 ), _V( 0.5446, -0.8387, 0.0 ) );
		pPanelMS->SetAngleRange( 300.0 );
		pPanelMS->SetCalibrationGroup( 4 );
		pPanelMS->SetMouseRegion( AID_R10, 0.679378f, 0.256269f, 0.752208f, 0.544727f );

		return;
	}

	void PanelR10::RegisterVC()
	{
		AtlantisPanel::RegisterVC();

		VECTOR3 ofs = STS()->GetOrbiterCoGOffset() + VC_OFFSET;

		oapiVCRegisterArea( AID_R10, PANEL_REDRAW_NEVER, PANEL_MOUSE_ONREPLAY | PANEL_MOUSE_LBDOWN | PANEL_MOUSE_LBUP | PANEL_MOUSE_LBPRESSED );
		oapiVCSetAreaClickmode_Quadrilateral( AID_R10,
			_V( 1.65494, 2.50958, 12.8038 ) + ofs, _V( 1.69245, 2.53394, 12.2564 ) + ofs,
			_V( 1.53864, 2.43406, 12.8038 ) + ofs, _V( 1.53913, 2.43437, 12.2564 ) + ofs );
		return;
	}

	void PanelR10::Realize()
	{
		DiscreteBundle* pLtgSw = STS()->BundleManager()->CreateBundle( "CABIN_LTG_SW", CABIN_LTG_SW_BUNDLE_LINES );
		pMSFlood->ConnectPort( 1, pLtgSw, CabinLighting::SW_MS_ON );
		DiscreteBundle* pLtgKnob = STS()->BundleManager()->CreateBundle( "CABIN_LTG_KNOB", CABIN_LTG_KNOB_BUNDLE_LINES );
		pMSFloodDim->Connect( pLtgKnob, CabinLighting::KNOB_MS );
		DiscreteBundle* pLtgPanel = STS()->BundleManager()->CreateBundle( "CABIN_LTG_PANEL", CABIN_LTG_PANEL_BUNDLE_LINES );
		pPanelMS->Connect( pLtgPanel, cabinlight::PNL_MS - 1 );

		AtlantisPanel::Realize();
		return;
	}
}
