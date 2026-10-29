/****************************************************************************
  This file is part of Space Shuttle Vessel

  The jets of the RCS (38 primary jets and 6 vernier jets, with the position, the direction and the thrust of each one), the selection
  of the jets for a rotation or a translation from their geometry, and the valve of each jet (no Orbiter calls, so it can be tested alone)

  The jet select of the real DAP uses tables that are loaded for each configuration of the jets; here the selection is computed from the
  geometry of the jets that work (the same principle: the jets whose couples turn the vehicle about the axis and whose forces cancel each
  other, or the jets whose forces go along the axis and whose moments cancel each other), so the choice follows the failures of the jets
  and the position of the center of mass.

  Data: positions and directions of the Orbiter Data Book (ParameterValues.h); 870 lbf (primary) and 24 lbf (vernier); minimum on-time
  80 ms; delays of the valves (on, off) of a sample I-load of the DAP: primary 34 and 22 ms, vernier 15 and 10 ms (a different source
  gives other values for other loads: they are nominal values here).

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
#ifndef _RCSJETS_H_
#define _RCSJETS_H_


#include <cstdint>
#include <cmath>
#include <cstring>
#include "ParameterValues.h"
#include "Dispersion.h"


namespace rcsjets
{
	inline constexpr int COUNT = 44;
	inline constexpr int PRIMARY_COUNT = 38;
	enum Tank {TANK_FWD = 0, TANK_LEFT, TANK_RIGHT};
	enum Axis {ROT_PITCH = 0, ROT_YAW, ROT_ROLL};// rotation (positive: nose up, nose right, right wing down)

	// valves and pulses
	inline constexpr double MIN_ON_TIME = 0.080;// [s] (primary and vernier, in pulse mode)
	inline constexpr double PRIMARY_DELAY_ON = 0.034, PRIMARY_DELAY_OFF = 0.022;// [s]
	inline constexpr double VERNIER_DELAY_ON = 0.015, VERNIER_DELAY_OFF = 0.010;// [s]
	inline constexpr double PRIMARY_ISP = 289.0, VERNIER_ISP = 265.0;// [s] (see the notes)

	struct Def
	{
		const char* name;
		VECTOR3 pos;// [m] in the frame of the vessel (the center of mass of the vessel without payloads)
		VECTOR3 dir;// direction of the force on the vehicle
		double thrust;// [N]
		int tank;
		bool vernier;
	};

#define SSV_JET( n ) {#n, RCS_##n##_OFS, RCS_##n##_DIR, RCS_##n##_THRUST0, (#n[0] == 'F') ? TANK_FWD : ((#n[0] == 'L') ? TANK_LEFT : TANK_RIGHT), (#n[1] == '5')}
	/** The 44 jets. */
	inline const Def* Jets( void )
	{
		static const Def jets[COUNT] = {
			// forward: 14 primary and 2 vernier
			SSV_JET( F1F ), SSV_JET( F2F ), SSV_JET( F3F ),
			SSV_JET( F1L ), SSV_JET( F3L ), SSV_JET( F2R ), SSV_JET( F4R ),
			SSV_JET( F1U ), SSV_JET( F2U ), SSV_JET( F3U ),
			SSV_JET( F1D ), SSV_JET( F2D ), SSV_JET( F3D ), SSV_JET( F4D ),
			SSV_JET( F5L ), SSV_JET( F5R ),
			// left pod: 12 primary and 2 vernier
			SSV_JET( L1A ), SSV_JET( L3A ),
			SSV_JET( L1L ), SSV_JET( L2L ), SSV_JET( L3L ), SSV_JET( L4L ),
			SSV_JET( L1U ), SSV_JET( L2U ), SSV_JET( L4U ),
			SSV_JET( L2D ), SSV_JET( L3D ), SSV_JET( L4D ),
			SSV_JET( L5D ), SSV_JET( L5L ),
			// right pod
			SSV_JET( R1A ), SSV_JET( R3A ),
			SSV_JET( R1R ), SSV_JET( R2R ), SSV_JET( R3R ), SSV_JET( R4R ),
			SSV_JET( R1U ), SSV_JET( R2U ), SSV_JET( R4U ),
			SSV_JET( R2D ), SSV_JET( R3D ), SSV_JET( R4D ),
			SSV_JET( R5D ), SSV_JET( R5R )
		};
		return jets;
	}
#undef SSV_JET

	inline uint64_t Bit( int jet ) { return 1ULL << jet; }

	/** @return the index of the jet with the name, or -1 */
	inline int Find( const char* name )
	{
		const Def* j = Jets();
		for (int i = 0; i < COUNT; i++)
		{
			if (strcmp( j[i].name, name ) == 0) return i;
		}
		return -1;
	}

	inline VECTOR3 Cross3( const VECTOR3& a, const VECTOR3& b )
	{
		return _V( (a.y * b.z) - (a.z * b.y), (a.z * b.x) - (a.x * b.z), (a.x * b.y) - (a.y * b.x) );
	}

	inline double Dot3( const VECTOR3& a, const VECTOR3& b ) { return (a.x * b.x) + (a.y * b.y) + (a.z * b.z); }

	inline double Len3( const VECTOR3& a ) { return sqrt( Dot3( a, a ) ); }

	inline VECTOR3 Force( int jet )
	{
		const Def& d = Jets()[jet];
		const double n = Len3( d.dir );
		return _V( d.dir.x, d.dir.y, d.dir.z ) * (d.thrust / n);
	}

	/** Moment of a jet about the center of mass [Nm] (cg: position of the center of mass in the frame of the vessel). */
	inline VECTOR3 Moment( int jet, const VECTOR3& cg )
	{
		return Cross3( Jets()[jet].pos - cg, Force( jet ) );
	}

	/** Axis of a rotation command (the sign is the direction of the command). */
	inline VECTOR3 RotationAxis( int axis, int sign )
	{
		const double s = (sign >= 0) ? 1.0 : -1.0;
		if (axis == ROT_PITCH) return _V( -s, 0.0, 0.0 );// nose up: the moment about -x
		if (axis == ROT_YAW) return _V( 0.0, s, 0.0 );// nose right
		return _V( 0.0, 0.0, -s );// right wing down
	}

	/** Direction of a translation command: 0 = forward (+z), 1 = right (+x), 2 = up (+y). */
	inline VECTOR3 TranslationAxis( int axis, int sign )
	{
		const double s = (sign >= 0) ? 1.0 : -1.0;
		if (axis == 0) return _V( 0.0, 0.0, s );
		if (axis == 1) return _V( s, 0.0, 0.0 );
		return _V( 0.0, s, 0.0 );
	}

	inline constexpr double LEVER = 15.0;// [m] to compare a force with a moment
	inline constexpr double INERTIA_X = 8.7e6, INERTIA_Y = 9.0e6, INERTIA_Z = 1.1e6;// [kg m^2] of an orbiter with the payload bay empty (about the pitch, yaw and roll axes)

	/** A moment weighted by the inverse of the inertia (relative to the pitch): what matters is the angular acceleration. */
	inline VECTOR3 Weighted( const VECTOR3& m )
	{
		return _V( m.x, m.y * (INERTIA_X / INERTIA_Y), m.z * (INERTIA_X / INERTIA_Z) );
	}

	/** Sum of the forces and of the moments of a set of jets. */
	inline void SetWrench( uint64_t set, const VECTOR3& cg, VECTOR3& force, VECTOR3& moment )
	{
		force = _V( 0.0, 0.0, 0.0 );
		moment = _V( 0.0, 0.0, 0.0 );
		for (int i = 0; i < COUNT; i++)
		{
			if ((set & Bit( i )) == 0) continue;
			force = force + Force( i );
			moment = moment + Moment( i, cg );
		}
	}

	namespace detail
	{
		inline int Candidates( bool rotation, const VECTOR3& axis, bool vernier, uint64_t excluded, const VECTOR3& cg, int* list, int maxCount )
		{
			double p[COUNT];
			double best = 0.0;
			for (int i = 0; i < COUNT; i++)
			{
				p[i] = 0.0;
				if ((Jets()[i].vernier != vernier) || ((excluded & Bit( i )) != 0)) continue;
				p[i] = rotation ? Dot3( Moment( i, cg ), axis ) : Dot3( Force( i ), axis );
				if (p[i] > best) best = p[i];
			}
			// the jets that help at least a fifth of the best one (the roll has short arms, the vernier jets small forces)
			double score[COUNT];
			int idx[COUNT];
			int n = 0;
			for (int i = 0; i < COUNT; i++)
			{
				if (p[i] <= (0.2 * best)) continue;
				int pos = n++;
				while ((pos > 0) && (score[pos - 1] < p[i]))
				{
					score[pos] = score[pos - 1];
					idx[pos] = idx[pos - 1];
					pos--;
				}
				score[pos] = p[i];
				idx[pos] = i;
			}
			if (n > maxCount) n = maxCount;
			for (int i = 0; i < n; i++) list[i] = idx[i];
			return n;
		}

		/** Value of a rotation set: moment about the axis, less the moments about the other axes and the net force (in moment units). */
		inline double RotationValue( uint64_t set, const VECTOR3& axis, const VECTOR3& cg )
		{
			VECTOR3 f, m;
			SetWrench( set, cg, f, m );
			const double along = Dot3( m, axis );
			const VECTOR3 perp = m - (axis * along);
			return along - (0.5 * Len3( perp )) - (0.5 * LEVER * Len3( f ));
		}

		/** Value of a translation set: force along the axis, less the other forces and the moments (in force units). */
		inline double TranslationValue( uint64_t set, const VECTOR3& axis, const VECTOR3& cg )
		{
			VECTOR3 f, m;
			SetWrench( set, cg, f, m );
			const double along = Dot3( f, axis );
			const VECTOR3 perp = f - (axis * along);
			return along - Len3( perp ) - (Len3( m ) / LEVER);
		}
	}

	/**
	 * Jets for a rotation: the set of up to k jets (of the primary ones or of the vernier ones) that turns the vehicle best about the axis with the
	 * least net force and the least moment about the other axes.
	 * @param excluded the jets that do not work or that are not selected (bits)
	 * @return the bits of the jets (0 if there is none)
	 */
	inline uint64_t SelectRotation( int axis, int sign, int k, bool vernier, uint64_t excluded, const VECTOR3& cg )
	{
		const VECTOR3 a = RotationAxis( axis, sign );
		int list[14];
		const int n = detail::Candidates( true, a, vernier, excluded, cg, list, 14 );
		if (n == 0) return 0;
		if (k > n) k = n;
		if (k < 1) k = 1;
		// all the combinations of up to k of the n candidates (at most 2^14): more jets only if they help (the roll has few good jets)
		VECTOR3 F[14], M[14];
		for (int i = 0; i < n; i++)
		{
			F[i] = Force( list[i] );
			M[i] = Weighted( Moment( list[i], cg ) );
		}
		uint32_t bestMask = 0;
		double bestValue = -1e300;
		for (uint32_t m = 1; m < (1u << n); m++)
		{
			int bits = 0;
			VECTOR3 f = _V( 0.0, 0.0, 0.0 ), mo = _V( 0.0, 0.0, 0.0 );
			for (int i = 0; i < n; i++)
			{
				if ((m >> i) & 1u)
				{
					bits++;
					f = f + F[i];
					mo = mo + M[i];
				}
			}
			if (bits > k) continue;
			const double along = Dot3( mo, a );
			const VECTOR3 perp = mo - (a * along);
			const double v = along - (0.5 * Len3( perp )) - (0.5 * LEVER * Len3( f ));
			if (v > bestValue)
			{
				bestValue = v;
				bestMask = m;
			}
		}
		uint64_t best = 0;
		for (int i = 0; i < n; i++)
		{
			if ((bestMask >> i) & 1u) best |= Bit( list[i] );
		}
		return best;
	}

	/**
	 * Jets for a translation: the subset of the primary jets whose forces go along the axis with the least moment (the number of jets is
	 * not limited: the ones that help are used).
	 */
	inline uint64_t SelectTranslation( int axis, int sign, uint64_t excluded, const VECTOR3& cg )
	{
		const VECTOR3 a = TranslationAxis( axis, sign );
		int list[14];
		const int n = detail::Candidates( false, a, false, excluded, cg, list, 14 );
		if (n == 0) return 0;
		VECTOR3 F[14], M[14];
		for (int i = 0; i < n; i++)
		{
			F[i] = Force( list[i] );
			M[i] = Weighted( Moment( list[i], cg ) );
		}
		uint32_t bestMask = 0;
		double bestValue = -1e300;
		for (uint32_t m = 1; m < (1u << n); m++)
		{
			VECTOR3 f = _V( 0.0, 0.0, 0.0 ), mo = _V( 0.0, 0.0, 0.0 );
			for (int i = 0; i < n; i++)
			{
				if ((m >> i) & 1u)
				{
					f = f + F[i];
					mo = mo + M[i];
				}
			}
			const double along = Dot3( f, a );
			const VECTOR3 perp = f - (a * along);
			const double v = along - Len3( perp ) - (Len3( mo ) / LEVER);
			if (v > bestValue)
			{
				bestValue = v;
				bestMask = m;
			}
		}
		uint64_t best = 0;
		for (int i = 0; i < n; i++)
		{
			if ((bestMask >> i) & 1u) best |= Bit( list[i] );
		}
		return best;
	}

	/**
	 * Jets that a rotation command of the DAP asks for. The level of the DAP (without the reduction of the thrust with time acceleration) is
	 * 1/3, 2/3 or 1 for 1, 2 or 3 couples of the primary jets (2 jets per couple, one more jet per couple for the roll: it has short arms)
	 * and 0.1 for the vernier jets (a couple).
	 * @param vernier output: true if the vernier jets are used
	 * @return the maximum number of jets of the selection
	 */
	inline int RotationJetCount( int axis, double level, bool& vernier )
	{
		vernier = (level <= 0.1001);
		if (vernier) return 2;
		int couples = static_cast<int>(floor( (level * 3.0) + 0.5 ));
		couples = (couples < 1) ? 1 : ((couples > 3) ? 3 : couples);
		return (axis == ROT_ROLL) ? (couples + 1) : (2 * couples);
	}

	/** Number of the jets of a set. */
	inline int Count( uint64_t set )
	{
		int n = 0;
		for (int i = 0; i < COUNT; i++) n += ((set >> i) & 1ULL) ? 1 : 0;
		return n;
	}

	/** Bits of the jets that are in the forward module (true) or in the pods (false). */
	inline uint64_t RegionMask( bool forward )
	{
		uint64_t m = 0;
		for (int i = 0; i < COUNT; i++)
		{
			if ((Jets()[i].tank == TANK_FWD) == forward) m |= Bit( i );
		}
		return m;
	}

	/** Moment of the vernier jets (2 of them) relative to the primary ones at the full level of the DAP (6 jets, 4 for the roll) about an axis: the DAP in the vernier mode uses it. */
	inline double VernierTorqueRatio( int axis, const VECTOR3& cg )
	{
		static double cache[3] = {-1.0, -1.0, -1.0};// the DAP asks every step: computed once
		if ((axis >= 0) && (axis < 3) && (cache[axis] >= 0.0)) return cache[axis];
		const VECTOR3 a = RotationAxis( axis, 1 );
		const uint64_t pri = SelectRotation( axis, 1, (axis == ROT_ROLL) ? 4 : 6, false, 0, cg );
		const uint64_t ver = SelectRotation( axis, 1, 2, true, 0, cg );
		VECTOR3 f, mp, mv;
		SetWrench( pri, cg, f, mp );
		SetWrench( ver, cg, f, mv );
		const double p = Dot3( mp, a );
		const double r = (p > 0.0) ? (Dot3( mv, a ) / p) : 0.1;
		if ((axis >= 0) && (axis < 3)) cache[axis] = r;
		return r;
	}

	/** Torque [N m] of the vernier jets (the set of 2 that the DAP selects) about an axis (mean of the two directions): the DAP in the vernier mode uses it. */
	inline double VernierTorque( int axis, const VECTOR3& cg )
	{
		static double cache[3] = {-1.0, -1.0, -1.0};// the DAP asks every step: computed once
		if ((axis >= 0) && (axis < 3) && (cache[axis] >= 0.0)) return cache[axis];
		double sum = 0.0;
		for (int sign = -1; sign <= 1; sign += 2)
		{
			const VECTOR3 a = RotationAxis( axis, sign );
			VECTOR3 f, m;
			SetWrench( SelectRotation( axis, sign, 2, true, 0, cg ), cg, f, m );
			sum += Dot3( m, a );
		}
		const double t = 0.5 * sum;
		if ((axis >= 0) && (axis < 3)) cache[axis] = t;
		return t;
	}

	// ---- the valve of a jet ----
	struct Valve
	{
		bool requested = false;// the command of the last step (before the minimum on-time)
		double held = 0.0;// [s] time that the command has been on (to keep it for the minimum on-time)
		bool commanded = false;// the command after the minimum on-time
		double delay = 0.0;// [s] time since the last change of the command (for the delay of the valve)
		bool open = false;// the valve is open (the thrust is on)
	};

	/**
	 * Advances a valve dt seconds with the command of the DAP.
	 * @param onDelay, offDelay delays of the valve to open and to close [s]
	 * @param minOn minimum time of the command [s]
	 * @return the share of the step with the thrust [0, 1]
	 */
	inline double StepValve( Valve& v, bool command, double dt, double onDelay, double offDelay, double minOn = MIN_ON_TIME )
	{
		if (dt <= 0.0) return v.open ? 1.0 : 0.0;
		// a command that starts is kept for the minimum on-time (the electronics of the jet do not allow shorter pulses)
		if (command && !v.requested) v.held = 0.0;
		v.requested = command;
		if (command || v.commanded) v.held += dt;
		const bool eff = command || (v.commanded && (v.held < minOn));
		if (eff != v.commanded)
		{
			v.commanded = eff;
			v.delay = 0.0;
		}
		v.delay += dt;// time since the change of the command, to the end of this step
		if (!eff && !v.commanded) v.held = 0.0;

		// the thrust follows the command after the delay of the valve
		const double wait = v.commanded ? onDelay : offDelay;
		const bool was = v.open;
		double onShare;
		if (v.commanded != v.open)
		{
			const double passed = v.delay;
			if (passed >= wait)
			{
				// the change happened during this step, at the time that the delay ended
				const double changeAt = dt - (passed - wait);
				const double t = (changeAt < 0.0) ? 0.0 : ((changeAt > dt) ? dt : changeAt);
				v.open = v.commanded;
				onShare = v.open ? ((dt - t) / dt) : (t / dt);
			}
			else onShare = was ? 1.0 : 0.0;
		}
		else onShare = was ? 1.0 : 0.0;
		return (onShare < 0.0) ? 0.0 : ((onShare > 1.0) ? 1.0 : onShare);
	}

	// ---- dispersions (1 sigma): the real ones are not published (estimates, see the notes of the page REALISM) ----
	inline constexpr double SIGMA_THRUST = 0.012;// relative; of each jet (the sample force vectors of the DAP document are 875-888 lbf for 870)
	inline constexpr double SIGMA_ISP = 0.007;// relative
	inline constexpr double SIGMA_DELAY = 0.12;// relative; of the delays of the valve
	inline constexpr double SIGMA_ANGLE = 0.3;// [deg]; installation of the jet (direction)
	inline constexpr double SIGMA_OFFSET = 0.005;// [m]; installation of the jet (position)
	inline constexpr double NOISE_THRUST = 0.008;// relative; at each firing
	inline constexpr double NOISE_ISP = 0.003;// relative; at each firing
	inline constexpr double NOISE_DELAY = 0.06;// relative; at each firing

	/** Parameters of a jet in the identifiers of the tolerances (disp::id::RCS_JET + 16 * jet + parameter). */
	enum TolParam {TOL_THRUST = 0, TOL_ISP, TOL_DELAY, TOL_DIR_X, TOL_DIR_Y, TOL_POS_X, TOL_POS_Y, TOL_POS_Z};

	/** Direction of a jet with the tolerance of its installation (a small turn about two axes). */
	inline VECTOR3 InstalledDirection( int jet, uint64_t seed, const disp::Mode& mode )
	{
		VECTOR3 d = Jets()[jet].dir;
		const double n = Len3( d );
		d = d * (1.0 / n);
		if (!mode.tolerances) return d;
		const uint32_t base = disp::id::RCS_JET + (16u * static_cast<uint32_t>(jet));
		const double ax = SIGMA_ANGLE * mode.scale * disp::Fixed( seed, base + TOL_DIR_X ) * 0.017453292519943295;
		const double ay = SIGMA_ANGLE * mode.scale * disp::Fixed( seed, base + TOL_DIR_Y ) * 0.017453292519943295;
		// a vector that is not parallel, to build two perpendicular axes
		const VECTOR3 helper = (fabs( d.x ) < 0.9) ? _V( 1.0, 0.0, 0.0 ) : _V( 0.0, 1.0, 0.0 );
		VECTOR3 u = Cross3( d, helper );
		u = u * (1.0 / Len3( u ));
		const VECTOR3 w = Cross3( d, u );
		VECTOR3 r = d + (u * ax) + (w * ay);
		return r * (1.0 / Len3( r ));
	}

	/** Position of a jet with the tolerance of its installation. */
	inline VECTOR3 InstalledPosition( int jet, uint64_t seed, const disp::Mode& mode )
	{
		VECTOR3 p = Jets()[jet].pos;
		if (!mode.tolerances) return p;
		const uint32_t base = disp::id::RCS_JET + (16u * static_cast<uint32_t>(jet));
		p.x += SIGMA_OFFSET * mode.scale * disp::Fixed( seed, base + TOL_POS_X );
		p.y += SIGMA_OFFSET * mode.scale * disp::Fixed( seed, base + TOL_POS_Y );
		p.z += SIGMA_OFFSET * mode.scale * disp::Fixed( seed, base + TOL_POS_Z );
		return p;
	}
}


#endif// _RCSJETS_H_
