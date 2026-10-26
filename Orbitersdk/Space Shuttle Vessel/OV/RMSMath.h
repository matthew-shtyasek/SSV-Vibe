/****************************************************************************
  This file is part of Space Shuttle Vessel

  Math of the RMS (frames, POR rotations and errors), without dependencies on the vessel, so it can be tested


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
#ifndef _RMSMATH_H_
#define _RMSMATH_H_


#include <OrbiterAPI.h>
#include <MathSSV.h>
#include <algorithm>
#include <cmath>


/**
 * Frames:
 * - local: Orbiter local frame (+X starboard, +Y up, +Z forward)
 * - body (B): Orbiter body axes used for the hand controller commands (+X tail, +Y port, +Z down), the same as the rotation axes of the RACS
 * - BACS: position frame of the displays (+X forward, +Y starboard, +Z down), in inches
 * - IK: frame of the inverse kinematics, the body frame rolled by the rollout angle of the RMS about X
 */
namespace RMSMath
{
	inline constexpr double IN2M_ = 0.0254;

	/**
	 * Factor of the rates of the arm for the mass of the captured payload (the unloaded arm is the reference): 1 for 500 kg or less, down to 0.1 for 29500 kg.
	 * @param massEffect false if the mass has no effect (option of the vessel)
	 * @param loadedRate true if the LOADED RATE override of the PDRS OVERRIDE display is on (the loaded rate limits halve the rates)
	 */
	inline double LoadScale( double payloadMass, bool massEffect, bool loadedRate )
	{
		double scale = 1.0;
		if (massEffect) scale = 1.0 - (0.9 * (payloadMass - 500.0) / 29000.0);
		if (loadedRate) scale *= 0.5;
		return (std::max)( 0.1, (std::min)( 1.0, scale ) );
	}

	/**
	 * Safing of the arm: the SAFING switch in SAFE brings the arm to rest, and the safing is maintained (also with the switch back in AUTO)
	 * until it is cancelled with the switch momentarily in CANCEL (or with SAFING CAN of the PDRS OVERRIDE display, which also masks SAFE).
	 * @param latched safing in progress (previous state)
	 * @return safing in progress
	 */
	inline bool SafingLatch( bool latched, bool safeSwitch, bool cancelSwitch, bool overrideCancel )
	{
		if (safeSwitch && !overrideCancel) return true;
		if (cancelSwitch || overrideCancel) return false;
		return latched;
	}

	/**
	 * The brakes hold the joints with the BRAKES switch in ON, and without power (the brakes are released with power on).
	 */
	inline bool BrakesApplied( bool brakesSwitchOn, bool powered )
	{
		return brakesSwitchOn || !powered;
	}

	/**
	 * The hand controllers and the keys operate the RMS when the view is the aft station of the RMS, and the RCS in every other place.
	 * The override (Ctrl+A) gives the opposite until the station is changed. Without an RMS, never.
	 */
	inline bool ControlsRMS( bool hasRMS, bool inRMSStation, bool override )
	{
		return hasRMS && (inRMSStation != override);
	}

	/** Local frame to body axes. */
	inline VECTOR3 LocalToBody( const VECTOR3& v )
	{
		return _V( -v.z, -v.x, -v.y );
	}

	/** Body axes to BACS, for a vector (no origin shift). */
	inline VECTOR3 BodyToBACS( const VECTOR3& v )
	{
		return _V( -v.x, -v.y, v.z );
	}

	/**
	 * Axis and angle of a rotation matrix.
	 * @param R rotation matrix
	 * @param axis unit vector, (1, 0, 0) if there is no rotation
	 * @param angle [rad], 0 to PI
	 */
	inline void AxisAngleFromMatrix( const MATRIX3& R, VECTOR3& axis, double& angle )
	{
		double c = (R.m11 + R.m22 + R.m33 - 1.0) * 0.5;
		c = (std::max)( -1.0, (std::min)( 1.0, c ) );// (the parentheses avoid the min and max macros of windows.h)
		angle = acos( c );
		axis = _V( R.m32 - R.m23, R.m13 - R.m31, R.m21 - R.m12 );
		double s = length( axis );
		if (s > 1e-9) axis = axis / s;
		else
		{
			axis = _V( 1.0, 0.0, 0.0 );
			if (angle < 1.0) angle = 0.0;// no rotation (an angle near 180 degrees would need the axis from the diagonal, not supported)
		}
	}

	/** Rotation matrix for a rotation of angle [rad] about a unit axis (right-handed). */
	inline MATRIX3 MatrixFromAxisAngle( const VECTOR3& axis, double angle )
	{
		const double c = cos( angle );
		const double sn = sin( angle );
		const double t = 1.0 - c;
		return _M( t * axis.x * axis.x + c, t * axis.x * axis.y - sn * axis.z, t * axis.x * axis.z + sn * axis.y,
				   t * axis.x * axis.y + sn * axis.z, t * axis.y * axis.y + c, t * axis.y * axis.z - sn * axis.x,
				   t * axis.x * axis.z - sn * axis.y, t * axis.y * axis.z + sn * axis.x, t * axis.z * axis.z + c );
	}

	/**
	 * Error between the current and the target state of the POR.
	 * @param cur current position (BACS) of the EE tip [in]
	 * @param curM current attitude of the EE (columns: x, y, z axes, in body axes)
	 * @param porOffset vector from the EE tip to the POR, in body axes [m] (zero for the EE tip)
	 * @param tgtpos target position of the POR (BACS) [in]
	 * @param tgtatt target attitude P, Y, R [deg]
	 * @param dB position error in body axes [m]
	 * @param axis axis of the rotation error (body axes)
	 * @param angle rotation error [rad]
	 */
	inline void PORError( const VECTOR3& cur, const MATRIX3& curM, const VECTOR3& porOffset, const double tgtpos[3], const double tgtatt[3], VECTOR3& dB, VECTOR3& axis, double& angle )
	{
		// the POR in BACS
		VECTOR3 por = cur + (BodyToBACS( porOffset ) / IN2M_);
		// position error in the body axes
		dB = _V( -(tgtpos[0] - por.x), -(tgtpos[1] - por.y), (tgtpos[2] - por.z) ) * IN2M_;

		// attitude error: rotation from the current to the target attitude
		MATRIX3 tgtM = GetRotationMatrixYZX( _V( tgtatt[2], tgtatt[0], tgtatt[1] ) * RAD );
		AxisAngleFromMatrix( mul( tgtM, Transpose( curM ) ), axis, angle );
	}

	/**
	 * Rotates the EE about the POR.
	 * @param angles rotation about X, Y and Z of the control axes [rad] (roll, pitch, yaw)
	 * @param axes control axes (body axes), the identity for the body axes
	 * @param usePOR rotate about the POR (porOffset) instead of the EE tip
	 * @param porOffset vector from the EE tip to the POR, in body axes [m]
	 * @param rollout rollout angle of the RMS [deg]
	 * @param dir EE direction (IK frame)
	 * @param rot EE rotation vector (IK frame)
	 * @param newPos EE position (IK frame), in/out
	 * @param newDir new EE direction (IK frame)
	 * @param newRot new EE rotation vector (IK frame)
	 */
	inline void RotateAboutPOR( const VECTOR3& angles, const VECTOR3 axes[3], bool usePOR, const VECTOR3& porOffset, double rollout, const VECTOR3& dir, const VECTOR3& rot, VECTOR3& newPos, VECTOR3& newDir, VECTOR3& newRot )
	{
		// rotates a vector (in the body axes) by the angles about the control axes
		auto rotate = [&]( const VECTOR3& v ) -> VECTOR3
		{
			VECTOR3 comp = _V( dotp( v, axes[0] ), dotp( v, axes[1] ), dotp( v, axes[2] ) );
			VECTOR3 rotated;
			RotateVector( comp, angles, rotated );
			return (axes[0] * rotated.x) + (axes[1] * rotated.y) + (axes[2] * rotated.z);
		};

		newDir = RotateVectorX( rotate( RotateVectorX( dir, -rollout ) ), rollout );
		newRot = RotateVectorX( rotate( RotateVectorX( rot, -rollout ) ), rollout );
		// rotating about a POR different from the EE tip moves the EE tip
		if (usePOR) newPos += RotateVectorX( porOffset - rotate( porOffset ), rollout );
	}
}

#endif// _RMSMATH_H_
