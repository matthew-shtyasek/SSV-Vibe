#include "DimmerKnob.h"
#include "../Atlantis.h"
#include "../LightingMath.h"
#include "../SSVOptions.h"
#include "../SettingsModel.h"
#include <cassert>


namespace vc
{
	constexpr double ROT_SPEED = 120.0;// rotation speed with the mouse button pressed [deg/s]
	constexpr double MAX_STEP = 0.1;// longest time step used for the rotation [s] (the real time step is used, so the knob also moves with the simulation paused)
	constexpr double TURNS = 720.0;// rotation of the animation [deg]: the pose is periodic, so one component with two turns covers every start angle


	DimmerKnob::DimmerKnob( Atlantis* _sts, const std::string& _ident, double defaultPos ):AtlantisVCComponent( _sts, _ident )
	{
		pos = defaultPos;
		ang = -1.0;
		offset = 0.0;
		calGroup = 0;
		lastOff = lastTrav = 0.0;
		lastRev = false;
		grpIndex = (UINT)-1;
		rotTravel = NULL;
		anim_rot = (UINT)-1;
		Registry().push_back( this );
	}

	DimmerKnob::~DimmerKnob()
	{
		std::vector<DimmerKnob*>& r = Registry();
		for (size_t i = 0; i < r.size(); i++)
		{
			if (r[i] == this)
			{
				r.erase( r.begin() + i );
				break;
			}
		}
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

		// the direction of the axis points to the user (the normal of the panel): a positive angle turns the knob clockwise as seen by the user (checked on the screenshots of O6 and A6U)
		// one component with two turns: the angle of the pointer is (offset - travel / 2 + travel * pos) from the rest pose of the mesh, modulo 360
		oapiWriteLogV( "(SSV_OV) [INFO] Knob %s: axis (%.3f %.3f %.3f), group %d", GetQualifiedIdentifier().c_str(), GetDirection().x, GetDirection().y, GetDirection().z, calGroup );
		rotTravel = new MGROUP_ROTATE( vc_idx, &grpIndex, 1, GetReference(), GetDirection(), static_cast<float>(TURNS * RAD) );
		anim_rot = STS()->CreateAnimation( 0.0 );
		STS()->AddAnimationComponent( anim_rot, 0.0, 1.0, rotTravel );

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
		if (anim_rot != (UINT)-1)
		{
			double off, trav;
			bool rev;
			GetParams( off, trav, rev );
			lastOff = off;
			lastTrav = trav;
			lastRev = rev;
			double th = fmod( (off - (0.5 * trav)) + (trav * (rev ? (1.0 - pos) : pos)), 360.0 );
			if (th < 0.0) th += 360.0;
			SetAnimation( anim_rot, th / TURNS );
		}
		return;
	}

	void DimmerKnob::GetParams( double& off, double& trav, bool& rev ) const
	{
		off = offset;
		trav = ang;
		rev = false;
		if ((calGroup >= 1) && (calGroup <= 4))
		{
			const int i = settings::KNOB_TOP_OFFSET + (3 * (calGroup - 1));
			const SSVOptions* o = STS()->GetOptions();
			off = o->Get( i );
			trav = o->Get( i + 1 );
			rev = (o->Get( i + 2 ) != 0.0);
		}
	}

	std::vector<DimmerKnob*>& DimmerKnob::Registry( void )
	{
		static std::vector<DimmerKnob*> knobs;
		return knobs;
	}

	void DimmerKnob::RefreshAll( void )
	{
		std::vector<DimmerKnob*>& r = Registry();
		for (size_t i = 0; i < r.size(); i++)
		{
			DimmerKnob* k = r[i];
			if (k->anim_rot == (UINT)-1) continue;
			double off, trav;
			bool rev;
			k->GetParams( off, trav, rev );
			if ((off != k->lastOff) || (trav != k->lastTrav) || (rev != k->lastRev)) k->OnPositionChange();
		}
	}

	bool DimmerKnob::OnMouseEvent( int _event, float x, float y )
	{
		if (_event & PANEL_MOUSE_LBPRESSED)
		{
			double dt = oapiGetSysStep();
			if (dt > MAX_STEP) dt = MAX_STEP;
			double off, trav;
			bool rev;
			GetParams( off, trav, rev );
			const double step = (ROT_SPEED * dt) / trav;
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
