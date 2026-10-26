#include "DimmerKnob.h"
#include "../Atlantis.h"
#include "../LightingMath.h"
#include <cassert>


namespace vc
{
	constexpr double ROT_SPEED = 120.0;// rotation speed with the mouse button pressed [deg/s]
	constexpr double MAX_STEP = 0.1;// longest time step used for the rotation [s] (the real time step is used, so the knob also moves with the simulation paused)
	constexpr double START_STATE = 0.001;// animation state of the first stop: the rotation to it is a separate component of the animation


	DimmerKnob::DimmerKnob( Atlantis* _sts, const std::string& _ident, double defaultPos ):AtlantisVCComponent( _sts, _ident )
	{
		pos = defaultPos;
		ang = -1.0;
		grpIndex = (UINT)-1;
		rotStart = NULL;
		rotTravel = NULL;
		anim_rot = (UINT)-1;
	}

	DimmerKnob::~DimmerKnob()
	{
		if (rotStart) delete rotStart;
		if (rotTravel) delete rotTravel;
	}

	bool DimmerKnob::OnParseLine( const char* line )
	{
		double tmp = -1.0;
		sscanf_s( line, "%lf", &tmp );
		if ((tmp >= 0.0) && (tmp <= 1.0)) pos = tmp;
		return true;
	}

	bool DimmerKnob::GetStateString( unsigned long ulBufferSize, char* pszBuffer )
	{
		sprintf_s( pszBuffer, ulBufferSize, "%lf", pos );
		return true;
	}

	void DimmerKnob::DefineGroup( UINT _grpIndex )
	{
		grpIndex = _grpIndex;
		return;
	}

	void DimmerKnob::SetAngleRange( double _ang )
	{
		assert( (_ang > 0.0) && (_ang < 360.0) && "DimmerKnob::SetAngleRange.ang" );
		ang = _ang;
		return;
	}

	void DimmerKnob::Connect( DiscreteBundle* pBundle, unsigned short usLine )
	{
		output.Connect( pBundle, usLine );
		return;
	}

	void DimmerKnob::DefineVCAnimations( UINT vc_idx )
	{
		assert( bHasReference && "DimmerKnob.bHasReference" );
		assert( bHasDirection && "DimmerKnob.bHasDirection" );
		assert( (ang > 0.0) && "DimmerKnob.ang" );

		// the direction of the axis points away from the user, so that a positive angle turns the knob clockwise
		// the group is in the middle of its travel: the first component takes it to the first stop (counterclockwise), the second to the other stop
		rotStart = new MGROUP_ROTATE( vc_idx, &grpIndex, 1, GetReference(), GetDirection(), static_cast<float>(-0.5 * ang * RAD) );
		rotTravel = new MGROUP_ROTATE( vc_idx, &grpIndex, 1, GetReference(), GetDirection(), static_cast<float>(ang * RAD) );
		anim_rot = STS()->CreateAnimation( START_STATE );
		STS()->AddAnimationComponent( anim_rot, 0.0, START_STATE, rotStart );
		STS()->AddAnimationComponent( anim_rot, START_STATE, 1.0, rotTravel );

		VerifyAnimations();
		OnPositionChange();
		return;
	}

	void DimmerKnob::Realize( void )
	{
		AtlantisVCComponent::Realize();
		output.SetLine( static_cast<float>(lightmath::KNOB_VOLTS * pos) );// the line is connected now
		return;
	}

	void DimmerKnob::OnPositionChange( void )
	{
		output.SetLine( static_cast<float>(lightmath::KNOB_VOLTS * pos) );
		if (anim_rot != (UINT)-1) SetAnimation( anim_rot, START_STATE + ((1.0 - START_STATE) * pos) );
		return;
	}

	bool DimmerKnob::OnMouseEvent( int _event, float x, float y )
	{
		if (_event & PANEL_MOUSE_LBPRESSED)
		{
			double dt = oapiGetSysStep();
			if (dt > MAX_STEP) dt = MAX_STEP;
			const double step = (ROT_SPEED * dt) / ang;
			if (x > 0.5)
			{
				// clockwise: brighter
				pos += step;
				if (pos > 1.0) pos = 1.0;
			}
			else
			{
				pos -= step;
				if (pos < 0.0) pos = 0.0;
			}
			OnPositionChange();
			return true;
		}
		return false;
	}
}
