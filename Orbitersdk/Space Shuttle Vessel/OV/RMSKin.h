/****************************************************************************
  This file is part of Space Shuttle Vessel

  Kinematics of the Remote Manipulator System: the geometry of the arm, the position of the end effector from the joint angles (forward) and the
  joint angles for a position of the end effector (inverse), with the arm deployed (no Orbiter calls, so it can be tested alone)

  The frame of the mesh is the one of the Orbiter body (+X to the starboard... see RMS.h for the frame of the calculations: +X towards the
  tail, +Y towards the port side, +Z down). The angles are in degrees, and the sign of the shoulder yaw joint is inverted (see CalcVectors).


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
#ifndef _RMSKIN_H_
#define _RMSKIN_H_


#include <OrbiterAPI.h>
#include <MathSSV.h>
#include <cmath>


namespace rmskin
{
	enum Joint {SHOULDER_YAW = 0, SHOULDER_PITCH = 1, ELBOW_PITCH = 2, WRIST_PITCH = 3, WRIST_YAW = 4, WRIST_ROLL = 5};

	inline constexpr double ROLLOUT_ANGLE = 19.48;// angle between the arm and the vertical when the arm is deployed [deg]

	// joint positions (stowed, MPM deployed), frame of the mesh
	inline const VECTOR3 SY_JOINT = _V( -2.641549, 0.422581, 6.9797 );
	inline const VECTOR3 SP_JOINT = _V( -2.7432, 0.709931, 6.9797 );
	inline const VECTOR3 EP_JOINT = _V( -2.692375, 0.566243, 0.60303 );
	inline const VECTOR3 WP_JOINT = _V( -2.7432, 0.709931, -6.4569 );
	inline const VECTOR3 WY_JOINT = _V( -2.7432, 0.709931, -6.9141 );
	inline const VECTOR3 EE_POS = _V( -2.7432, 0.709931, -8.3365 );

	inline const VECTOR3 SY_AXIS = _V( -0.333478, 0.942758, 0.0 );
	inline const VECTOR3 SP_AXIS = _V( 0.942758, 0.333478, 0.0 );
	inline const VECTOR3 EP_AXIS = SP_AXIS;
	inline const VECTOR3 WP_AXIS = SP_AXIS;
	inline const VECTOR3 WY_AXIS = SY_AXIS;
	inline const VECTOR3 WR_AXIS = _V( 0.0, 0.0, 1.0 );
	inline const VECTOR3 EE_ROT = _V( 0.0, 1.0, 0.0 );

	inline const double SP_EP_DIST = length( EP_JOINT - SP_JOINT );
	inline const double EP_WP_DIST = length( WP_JOINT - EP_JOINT );
	inline const double WP_WY_DIST = length( WY_JOINT - WP_JOINT );
	inline const double WY_EE_DIST = length( WY_JOINT - EE_POS );
	inline const double SP_NULL_ANGLE = DEG * acos( (SP_JOINT.z - EP_JOINT.z) / SP_EP_DIST );// effective angle of the shoulder boom in the null position [deg]
	inline const double EP_NULL_ANGLE = DEG * acos( (EP_JOINT.z - WP_JOINT.z) / EP_WP_DIST );// effective angle of the elbow boom in the null position [deg]

	// limits of the joints [deg]: hard stops, soft stops (the software stops), reach limits (the caution of the software)
	inline constexpr double JOINT_LIMITS[2][6] = {{-180.0, -2.0, -161.0, -121.4, -121.3, -447.0},
		{+180.0, +145.0, +2.4, +121.4, +121.3, +447.0}};
	inline constexpr double JOINT_SOFTSTOPS[2][6] = {{-177.4, +0.6, -157.6, -116.4, -116.6, -442.0},
		{+177.4, +142.4, -0.4, +116.4, +116.6, +442.0}};
	inline constexpr double JOINT_REACHLIMITS[2][6] = {{-175.4, +2.6, -155.6, -114.4, -114.6, -440.0},
		{+175.4, +140.4, -2.4, +114.4, +114.6, +440.0}};

	/** Convert a vector from the Orbiter frame to the RMS frame (does not rotate it by the rollout angle). */
	inline VECTOR3 ToRMSFrame( const VECTOR3& v )
	{
		return _V( -v.z, -v.x, -v.y );
	}

	/** Pose of the end effector from the joint angles, with the arm deployed. */
	struct Pose
	{
		VECTOR3 posEE;// position of the tip, frame of the mesh
		VECTOR3 dirEE;// direction of the end effector
		VECTOR3 rotEE;// roll vector
		VECTOR3 rotEEik;// roll vector for the inverse kinematics
		VECTOR3 posWP, posWY, posEP, posSP;// positions of some joints
	};

	/**
	 * Forward kinematics (CalcVectors of the RMS without the rollout of the MPM).
	 * @param q joint angles [deg]
	 */
	inline Pose Forward( const double q[6] )
	{
		VECTOR3 posSY = SY_JOINT;
		VECTOR3 axisSY = SY_AXIS;

		VECTOR3 posSP = SP_JOINT;
		VECTOR3 axisSP = SP_AXIS;

		VECTOR3 posEP = EP_JOINT;
		VECTOR3 axisEP = EP_AXIS;

		VECTOR3 posWP = WP_JOINT;
		VECTOR3 axisWP = WP_AXIS;

		VECTOR3 posWY = WY_JOINT;
		VECTOR3 axisWY = WY_AXIS;

		VECTOR3 posWR = WY_JOINT;
		VECTOR3 axisWR = WR_AXIS;

		VECTOR3 posEE = EE_POS;
		VECTOR3 rotEE = EE_ROT;
		VECTOR3 dirEE;
		VECTOR3 rotEEik;

		// SY rotation
		MATRIX3 mrotSY = rotm( axisSY, -q[SHOULDER_YAW] * RAD );// HACK fix wrong angle sign

		posSP = mul( mrotSY, posSP - posSY ) + posSY;
		axisSP = mul( mrotSY, axisSP );

		posEP = mul( mrotSY, posEP - posSY ) + posSY;
		axisEP = mul( mrotSY, axisEP );

		posWP = mul( mrotSY, posWP - posSY ) + posSY;
		axisWP = mul( mrotSY, axisWP );

		posWY = mul( mrotSY, posWY - posSY ) + posSY;
		axisWY = mul( mrotSY, axisWY );

		posWR = mul( mrotSY, posWR - posSY ) + posSY;
		axisWR = mul( mrotSY, axisWR );

		posEE = mul( mrotSY, posEE - posSY ) + posSY;
		rotEE = mul( mrotSY, rotEE );

		// SP rotation
		MATRIX3 mrotSP = rotm( axisSP, q[SHOULDER_PITCH] * RAD );

		posEP = mul( mrotSP, posEP - posSP ) + posSP;
		axisEP = mul( mrotSP, axisEP );

		posWP = mul( mrotSP, posWP - posSP ) + posSP;
		axisWP = mul( mrotSP, axisWP );

		posWY = mul( mrotSP, posWY - posSP ) + posSP;
		axisWY = mul( mrotSP, axisWY );

		posWR = mul( mrotSP, posWR - posSP ) + posSP;
		axisWR = mul( mrotSP, axisWR );

		posEE = mul( mrotSP, posEE - posSP ) + posSP;
		rotEE = mul( mrotSP, rotEE );

		// EP rotation
		MATRIX3 mrotEP = rotm( axisEP, q[ELBOW_PITCH] * RAD );

		posWP = mul( mrotEP, posWP - posEP ) + posEP;
		axisWP = mul( mrotEP, axisWP );

		posWY = mul( mrotEP, posWY - posEP ) + posEP;
		axisWY = mul( mrotEP, axisWY );

		posWR = mul( mrotEP, posWR - posEP ) + posEP;
		axisWR = mul( mrotEP, axisWR );

		posEE = mul( mrotEP, posEE - posEP ) + posEP;
		rotEE = mul( mrotEP, rotEE );

		// WP rotation
		MATRIX3 mrotWP = rotm( axisWP, q[WRIST_PITCH] * RAD );

		posWY = mul( mrotWP, posWY - posWP ) + posWP;
		axisWY = mul( mrotWP, axisWY );

		posWR = mul( mrotWP, posWR - posWP ) + posWP;
		axisWR = mul( mrotWP, axisWR );

		posEE = mul( mrotWP, posEE - posWP ) + posWP;
		rotEE = mul( mrotWP, rotEE );

		// WY rotation
		MATRIX3 mrotWY = rotm( axisWY, q[WRIST_YAW] * RAD );

		posWR = mul( mrotWY, posWR - posWY ) + posWY;
		axisWR = mul( mrotWY, axisWR );

		posEE = mul( mrotWY, posEE - posWY ) + posWY;
		rotEE = mul( mrotWY, rotEE );

		// WR rotation
		MATRIX3 mrotWR = rotm( axisWR, q[WRIST_ROLL] * RAD );
		rotEE = mul( mrotWR, rotEE );
		{
			MATRIX3 mrotIK = rotm( axisWR, ROLLOUT_ANGLE * RAD );
			rotEEik = -mul( mrotIK, rotEE );
		}
		dirEE = -axisWR;

		Pose p;
		p.posEE = posEE;
		p.dirEE = dirEE;
		p.rotEE = rotEE;
		p.rotEEik = rotEEik;
		p.posWP = posWP;
		p.posWY = posWY;
		p.posEP = posEP;
		p.posSP = posSP;
		return p;
	}

	/** Position, direction and roll vector of the end effector in the frame of the inverse kinematics (the frame of the arm). */
	struct IKPose
	{
		VECTOR3 pos, dir, rot;
	};

	/** The pose of the end effector in the frame of the inverse kinematics from the pose of the forward kinematics. */
	inline IKPose ToIKFrame( const Pose& p )
	{
		IKPose ik;
		ik.dir = ToRMSFrame( RotateVectorZ( p.dirEE, -ROLLOUT_ANGLE ) );
		ik.rot = ToRMSFrame( RotateVectorZ( p.rotEEik, -ROLLOUT_ANGLE ) );
		ik.pos = ToRMSFrame( RotateVectorZ( p.posEE - SP_JOINT, -ROLLOUT_ANGLE ) );
		return ik;
	}

	/** Pose of the end effector in the frame of the inverse kinematics from the joint angles [deg]. */
	inline IKPose ForwardIK( const double q[6] )
	{
		return ToIKFrame( Forward( q ) );
	}

	/**
	 * Inverse kinematics: the joint angles for a pose of the end effector.
	 * @param newPos position in the IK frame
	 * @param newDir direction of the end effector in the IK frame
	 * @param newRot roll vector in the IK frame
	 * @param current current joint angles [deg] (the solution closest to them is used where there are two)
	 * @param[out] angles joint angles [deg]
	 * @return false if the pose is not reachable
	 */
	inline bool Inverse( const VECTOR3& newPos, const VECTOR3& newDir, const VECTOR3& newRot, const double current[6], double angles[6] )
	{
		// NOTE: for cross products, |a x b|=|a||b|sin(theta)
		// all vectors obtained as cross products should be normalized

		VECTOR3 arm_wy_cpos = newPos - newDir * WY_EE_DIST;
		double yaw_angle = atan2( arm_wy_cpos.y, arm_wy_cpos.x );

		VECTOR3 boom_plane = _V( cos( yaw_angle ), sin( yaw_angle ), 0.0 );
		// vector in the XY plane and perpendicular to the arm booms
		VECTOR3 normal = crossp( boom_plane, _V( 0.0, 0.0, 1.0 ) );
		normal /= length( normal );

		if (((newDir.z > normal.z) && (newDir.z < boom_plane.z)) || ((newDir.z < normal.z) && (newDir.z > boom_plane.z)))
			angles[WRIST_YAW] = -DEG * acos( dotp( normal, newDir ) ) + 90.0;
		else angles[WRIST_YAW] = -90.0 + DEG * acos( dotp( normal, newDir ) );

		double phi;
		// vector in the same plane as the arm booms and perpendicular to the direction of the end effector
		VECTOR3 wp_normal = crossp( normal, newDir );
		if (Eq( length( wp_normal ), 0.0, 0.001 ))// the direction is perpendicular to the arm booms
		{
			// use the same phi angle as for the previous position
			phi = current[SHOULDER_PITCH] + current[ELBOW_PITCH] + current[WRIST_PITCH];
			wp_normal = RotateVectorZ( boom_plane, phi + 90.0 );
		}
		else
		{
			wp_normal /= length( wp_normal );
			phi = DEG * acos( wp_normal.z );
			if (newDir.z > 0.0) phi = -phi;
		}

		angles[WRIST_ROLL] = -acos( dotp( wp_normal, newRot ) ) * DEG;

		VECTOR3 cross_product = crossp( wp_normal, newRot );
		if (Eq( 1.0, dotp( cross_product / length( cross_product ), newDir ), 0.05 ))
			angles[WRIST_ROLL] = -angles[WRIST_ROLL];

		VECTOR3 arm_wp_dir = crossp( wp_normal, normal );// wp_normal and normal are perpendicular
		VECTOR3 arm_wp_cpos = arm_wy_cpos - arm_wp_dir * WP_WY_DIST;
		double r = length( arm_wp_cpos );

		angles[SHOULDER_YAW] = -DEG * yaw_angle;
		double rho = sqrt( (arm_wp_cpos.x * arm_wp_cpos.x) + (arm_wp_cpos.y * arm_wp_cpos.y) );
		double cos_phibar_e = ((r * r) - (SP_EP_DIST * SP_EP_DIST) - (EP_WP_DIST * EP_WP_DIST)) / (-2 * SP_EP_DIST * EP_WP_DIST);
		if (fabs( cos_phibar_e ) > 1) return false;// the elbow can not reach the point
		angles[ELBOW_PITCH] = (DEG * acos( cos_phibar_e )) - 180.0 - EP_NULL_ANGLE - SP_NULL_ANGLE;
		double cos_phi_s2 = ((EP_WP_DIST * EP_WP_DIST) - (SP_EP_DIST * SP_EP_DIST) - (r * r)) / (-2 * SP_EP_DIST * r);
		if (fabs( cos_phi_s2 ) > 1) return false;// the shoulder can not reach the point
		angles[SHOULDER_PITCH] = (DEG * (-atan2( arm_wp_cpos.z, rho ) + acos( cos_phi_s2 ))) + SP_NULL_ANGLE;

		angles[WRIST_PITCH] = phi - angles[SHOULDER_PITCH] - angles[ELBOW_PITCH];

		// the full range of the wrist roll joint
		if ((current[WRIST_ROLL] - angles[WRIST_ROLL]) > 180.0)
		{
			angles[WRIST_ROLL] += 360.0;
		}
		else if ((current[WRIST_ROLL] - angles[WRIST_ROLL]) < -180.0)
		{
			angles[WRIST_ROLL] -= 360.0;
		}
		return true;
	}
}


#endif// _RMSKIN_H_
