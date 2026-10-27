/****************************************************************************
  This file is part of Space Shuttle Vessel

  Mass properties of the orbiter with the RMS: center of mass and moments of inertia when the arm and the payload that it holds
  are out of the cradle (no Orbiter calls, so it can be tested alone)

  The arm is modelled as five point masses (shoulder, upper boom, elbow, lower boom, wrist and end effector) at the places given
  by the joints, with the total mass of the real arm. The orbiter already includes the arm in its stowed position (the default
  center of mass), so the arm is added as a displacement: the mass at the current place and the same mass, negative, at the
  stowed place. The moments of inertia of the pieces about their own centers are not used (small compared with the distances).

  Orbiter gives the moments of inertia of a vessel divided by its mass (PMI, in m^2), about the axes of the vessel frame, so the
  functions here use the same convention.


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
#ifndef _MASSPROPS_H_
#define _MASSPROPS_H_


#include <OrbiterAPI.h>
#include <cmath>


namespace massprops
{
	inline constexpr double ARM_MASS = 410.5;// [kg] mass of the RMS (905 lb)

	// the share of the mass of the arm in each piece
	inline constexpr double SHARE_SHOULDER = 0.20;// at the shoulder pitch joint (the joints, the motors and the MPM hardware)
	inline constexpr double SHARE_UPPER_BOOM = 0.28;// in the middle of the upper arm boom
	inline constexpr double SHARE_ELBOW = 0.10;// at the elbow joint
	inline constexpr double SHARE_LOWER_BOOM = 0.26;// in the middle of the lower arm boom
	inline constexpr double SHARE_WRIST = 0.16;// between the wrist pitch joint and the end effector

	/**
	 * Mass [kg] at a place [m]. Negative masses are allowed (they take mass away from a place).
	 * I is the moment of inertia of the mass about its own center, about axes parallel to the frame [kg m^2] (zero for a point mass; a body
	 * made of several pieces, like the arm, has the inertia of its pieces about its center here, with the sign of the mass).
	 */
	struct Point
	{
		double m;
		VECTOR3 r;
		VECTOR3 I;
	};

	/** Moment of inertia of a thin uniform rod from a to b about the axes through its center, parallel to the frame [kg m^2] (the products of inertia are dropped). */
	inline VECTOR3 RodInertia( double m, const VECTOR3& a, const VECTOR3& b )
	{
		const VECTOR3 d = b - a;
		const double l2 = (d.x * d.x) + (d.y * d.y) + (d.z * d.z);
		if (l2 <= 0.0) return _V( 0.0, 0.0, 0.0 );
		const double k = m / 12.0;
		return _V( k * (l2 - (d.x * d.x)), k * (l2 - (d.y * d.y)), k * (l2 - (d.z * d.z)) );
	}

	inline VECTOR3 ArmCoM( const VECTOR3& shoulderPitch, const VECTOR3& elbowPitch, const VECTOR3& wristPitch, const VECTOR3& endEffector );

	/**
	 * Inertia of the arm, made of its links, about its center of mass: the shoulder, the elbow as points, the upper boom (shoulder pitch
	 * to elbow pitch), the lower boom (elbow pitch to wrist pitch) and the wrist with the end effector as uniform rods.
	 */
	inline VECTOR3 ArmInertia( const VECTOR3& shoulderPitch, const VECTOR3& elbowPitch, const VECTOR3& wristPitch, const VECTOR3& endEffector )
	{
		const VECTOR3 com = ArmCoM( shoulderPitch, elbowPitch, wristPitch, endEffector );
		const double mUpper = ARM_MASS * SHARE_UPPER_BOOM;
		const double mLower = ARM_MASS * SHARE_LOWER_BOOM;
		const double mWrist = ARM_MASS * SHARE_WRIST;
		VECTOR3 inertia = RodInertia( mUpper, shoulderPitch, elbowPitch ) + RodInertia( mLower, elbowPitch, wristPitch ) + RodInertia( mWrist, wristPitch, endEffector );
		// the pieces about the center of the arm (parallel axes)
		const double mShoulder = ARM_MASS * SHARE_SHOULDER;
		const double mElbow = ARM_MASS * SHARE_ELBOW;
		const VECTOR3 pieceR[5] = {shoulderPitch, (shoulderPitch + elbowPitch) * 0.5, elbowPitch, (elbowPitch + wristPitch) * 0.5, (wristPitch + endEffector) * 0.5};
		const double pieceM[5] = {mShoulder, mUpper, mElbow, mLower, mWrist};
		for (int i = 0; i < 5; i++)
		{
			const VECTOR3 d = pieceR[i] - com;
			inertia += _V( pieceM[i] * ((d.y * d.y) + (d.z * d.z)), pieceM[i] * ((d.x * d.x) + (d.z * d.z)), pieceM[i] * ((d.x * d.x) + (d.y * d.y)) );
		}
		return inertia;
	}

	/** Center of mass of the arm for the positions of its joints and of the end effector (all in the same frame). */
	inline VECTOR3 ArmCoM( const VECTOR3& shoulderPitch, const VECTOR3& elbowPitch, const VECTOR3& wristPitch, const VECTOR3& endEffector )
	{
		return (shoulderPitch * SHARE_SHOULDER)
			+ (((shoulderPitch + elbowPitch) * 0.5) * SHARE_UPPER_BOOM)
			+ (elbowPitch * SHARE_ELBOW)
			+ (((elbowPitch + wristPitch) * 0.5) * SHARE_LOWER_BOOM)
			+ (((wristPitch + endEffector) * 0.5) * SHARE_WRIST);
	}

	/** Mass and center of mass of a set of points. The center is the origin if the total mass is zero. */
	inline double Combine( const Point* p, int n, VECTOR3& cog )
	{
		double mass = 0.0;
		VECTOR3 sum = _V( 0.0, 0.0, 0.0 );
		for (int i = 0; i < n; i++)
		{
			mass += p[i].m;
			sum += p[i].r * p[i].m;
		}
		cog = (mass != 0.0) ? (sum / mass) : _V( 0.0, 0.0, 0.0 );
		return mass;
	}

	/** Moment of inertia of a point mass about the axes through a point, parallel to the axes of the frame: [kg m^2] about x, y and z. */
	inline VECTOR3 PointInertia( double m, const VECTOR3& r, const VECTOR3& about )
	{
		const VECTOR3 d = r - about;
		return _V( m * ((d.y * d.y) + (d.z * d.z)), m * ((d.x * d.x) + (d.z * d.z)), m * ((d.x * d.x) + (d.y * d.y)) );
	}

	/**
	 * PMI [m^2] of a vessel (mass and PMI given, about its center of mass baseCoG) with points added, about the new center of mass.
	 * The points can have negative mass to take mass away (an arm that left its place in the vessel).
	 * @param basePMI PMI of the vessel before the change (Orbiter's convention: moment of inertia divided by mass)
	 * @param baseMass mass of the vessel before the change [kg]
	 * @param baseCoG center of mass of the vessel before the change
	 * @param p points added
	 * @param n number of points
	 * @param[out] cog center of mass after the change
	 * @param[out] mass mass after the change [kg]
	 */
	inline VECTOR3 ComposePMI( const VECTOR3& basePMI, double baseMass, const VECTOR3& baseCoG, const Point* p, int n, VECTOR3& cog, double& mass )
	{
		VECTOR3 sum = baseCoG * baseMass;
		mass = baseMass;
		for (int i = 0; i < n; i++)
		{
			mass += p[i].m;
			sum += p[i].r * p[i].m;
		}
		if (mass <= 0.0)
		{
			cog = baseCoG;
			mass = baseMass;
			return basePMI;
		}
		cog = sum / mass;

		VECTOR3 inertia = _V( baseMass * basePMI.x, baseMass * basePMI.y, baseMass * basePMI.z ) + PointInertia( baseMass, baseCoG, cog );
		for (int i = 0; i < n; i++) inertia += PointInertia( p[i].m, p[i].r, cog ) + p[i].I;
		return inertia / mass;
	}

	/**
	 * Increase of the PMI from points whose mass is already in the mass of the vessel (the center of mass is kept where it is).
	 * @param p points (the sum of their masses may be zero: an arm displaced)
	 * @param n number of points
	 * @param about center of mass of the vessel
	 * @param totalMass mass of the vessel [kg]
	 */
	inline VECTOR3 AddedPMI( const Point* p, int n, const VECTOR3& about, double totalMass )
	{
		VECTOR3 inertia = _V( 0.0, 0.0, 0.0 );
		if (totalMass <= 0.0) return inertia;
		for (int i = 0; i < n; i++) inertia += PointInertia( p[i].m, p[i].r, about ) + p[i].I;
		return inertia / totalMass;
	}
}


#endif// _MASSPROPS_H_
