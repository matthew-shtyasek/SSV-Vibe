/****************************************************************************
  This file is part of Space Shuttle Vessel

  Dispersions of the parts: tolerances of the vehicle (fixed values of every part, from a seed that is saved with the vehicle) and noise
  (values that change at each use or firing). No Orbiter calls, so it can be tested alone.

  Two layers, as in the real hardware:
  - the tolerances: the part made, installed and adjusted for this vehicle is a little different from the nominal one (thrust of a jet,
    set point of a valve, angle of the installation). The value of every part is a function of the seed of the vehicle and of an
    identifier of the part, so it does not change from one run to the next and only the seed is saved
  - the noise: quick variations over them at each use (the same jet does not give the same impulse twice)

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
#ifndef _DISPERSION_H_
#define _DISPERSION_H_


#include <cmath>
#include <cstdint>


namespace disp
{
	/** The values are limited to this number of standard deviations (a part out of the limits of acceptance is not installed). */
	inline constexpr double TRUNCATION = 3.0;

	/** Which dispersions are on, and their scale (1 = estimated real values). */
	struct Mode
	{
		bool tolerances = false;
		bool noise = false;
		double scale = 1.0;
	};

	/** Next number of a generator of 64 bits (SplitMix64: any seed is good, the numbers do not repeat for 2^64 steps). */
	inline uint64_t SplitMix( uint64_t& state )
	{
		state += 0x9E3779B97F4A7C15ULL;
		uint64_t z = state;
		z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
		z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
		return z ^ (z >> 31);
	}

	/** Number in (0, 1) from 64 bits (never 0, so the logarithm is defined). */
	inline double Unit( uint64_t bits )
	{
		return (static_cast<double>(bits >> 11) + 0.5) * (1.0 / 9007199254740992.0);
	}

	/** Standard normal number from two generated ones (Box-Muller), limited to +-TRUNCATION. */
	inline double NormalFrom( uint64_t& state )
	{
		const double u1 = Unit( SplitMix( state ) );
		const double u2 = Unit( SplitMix( state ) );
		const double z = sqrt( -2.0 * log( u1 ) ) * cos( 6.283185307179586 * u2 );
		return (z < -TRUNCATION) ? -TRUNCATION : ((z > TRUNCATION) ? TRUNCATION : z);
	}

	/**
	 * Tolerance of a part of the vehicle: a standard normal number (limited) that depends only on the seed of the vehicle and on the
	 * identifier of the part.
	 */
	inline double Fixed( uint64_t seed, uint32_t id )
	{
		uint64_t state = seed ^ (0xD1B54A32D192ED03ULL * (static_cast<uint64_t>(id) + 1ULL));
		SplitMix( state );// the first numbers of near seeds are alike
		return NormalFrom( state );
	}

	/** Noise: a generator of its own (it is not saved, so every run is different). */
	class Noise
	{
	public:
		explicit Noise( uint64_t seed = 0x2545F4914F6CDD1DULL ) : state( seed ) {}

		void Seed( uint64_t seed ) { state = seed; }

		/** Standard normal number, limited to +-TRUNCATION. */
		double Normal( void ) { return NormalFrom( state ); }

		/** Number in (0, 1). */
		double Uniform( void ) { return Unit( SplitMix( state ) ); }

	private:
		uint64_t state;
	};

	/** A value with a relative dispersion: nominal * (1 + sigma * scale * z). */
	inline double Rel( double nominal, double sigma, double z, double scale = 1.0 )
	{
		return nominal * (1.0 + (sigma * scale * z));
	}

	/** A value with an absolute dispersion: nominal + sigma * scale * z. */
	inline double Abs( double nominal, double sigma, double z, double scale = 1.0 )
	{
		return nominal + (sigma * scale * z);
	}

	/** Identifiers of the parts for Fixed(): the first of each group, the others follow it (system, jet or elevon number). */
	namespace id
	{
		inline constexpr uint32_t HYD_ELEVON_SWITCH = 100;// + elevon (0-3)
		inline constexpr uint32_t HYD_ELEVON_RATE = 110;// + elevon (0-3)
		inline constexpr uint32_t HYD_RSB_RATE = 120;
		inline constexpr uint32_t HYD_BODYFLAP_RATE = 121;
		inline constexpr uint32_t HYD_BRAKE_GAIN = 122;
		inline constexpr uint32_t HYD_PUMP_SET = 130;// + system (0-2)
		inline constexpr uint32_t HYD_PUMP_LOW = 133;// + system (0-2)
		inline constexpr uint32_t HYD_WSB = 200;// + 16 * system (0-2) + the parameter (see HydModel.h)
		inline constexpr uint32_t RCS_JET = 1000;// + 16 * jet + the parameter (see RCSJets.h)
	}

	/** Seed for a vehicle from its name (the same vehicle gets the same seed when it has none saved). */
	inline uint64_t SeedFromText( const char* text )
	{
		uint64_t h = 0xCBF29CE484222325ULL;// FNV-1a
		while ((text != nullptr) && (*text != 0))
		{
			h ^= static_cast<unsigned char>(*text++);
			h *= 0x100000001B3ULL;
		}
		uint64_t state = h;
		return SplitMix( state );
	}
}


#endif// _DISPERSION_H_
