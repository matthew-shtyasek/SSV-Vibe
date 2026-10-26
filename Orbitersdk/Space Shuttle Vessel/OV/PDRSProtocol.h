/****************************************************************************
  This file is part of Space Shuttle Vessel

  PDRS software: the data that the MCIU and the GPC exchange (words of 16 bits), without Orbiter calls so it can be tested alone

  In the real vehicle the Manipulator Controller Interface Unit (MCIU) puts the positions of the hand controllers and switches of the D&C panel and
  the data of the joints on the data bus, and receives the commands of the joint rates from the GPC. The software of the GPC does not see anything
  else of the arm. The same is done here: the arm gives a frame of words to the software and gets a frame of words back.


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
#ifndef _PDRSPROTOCOL_H_
#define _PDRSPROTOCOL_H_


#include "PDRSLogic.h"
#include <cstring>


namespace pdrs
{
	inline constexpr int FRAME_IN_WORDS = 48;// MCIU to GPC
	inline constexpr int FRAME_OUT_WORDS = 10;// GPC to MCIU
	inline constexpr double GPC_CYCLE = 0.08;// [s] the software runs at 12.5 Hz
	inline constexpr double COMM_TIMEOUT = 0.5;// [s] the MCIU stops the arm if the software does not answer

	/** The frames that are on the bus. */
	struct Mailbox
	{
		unsigned short toGpc[FRAME_IN_WORDS];
		unsigned short fromGpc[FRAME_OUT_WORDS];
		unsigned short counterIn;// the counters tell new frames
		unsigned short counterOut;
		unsigned short counterRead;// incremented by the software when it has read a frame

		Mailbox( void ) { memset( this, 0, sizeof( *this ) ); }
	};

	namespace detail
	{
		inline void PutS16( unsigned short* w, double value, double scale )
		{
			double v = value * scale;
			if (v > 32767.0) v = 32767.0;
			if (v < -32768.0) v = -32768.0;
			*w = static_cast<unsigned short>(static_cast<short>(floor( v + 0.5 )));
		}

		inline double GetS16( unsigned short w, double scale )
		{
			return static_cast<short>(w) / scale;
		}

		inline void PutS32( unsigned short* w, double value, double scale )
		{
			double v = value * scale;
			if (v > 2147483647.0) v = 2147483647.0;
			if (v < -2147483648.0) v = -2147483648.0;
			const unsigned int u = static_cast<unsigned int>(static_cast<int>(floor( v + 0.5 )));
			w[0] = static_cast<unsigned short>(u & 0xFFFF);
			w[1] = static_cast<unsigned short>(u >> 16);
		}

		inline double GetS32( const unsigned short* w, double scale )
		{
			const unsigned int u = static_cast<unsigned int>(w[0]) | (static_cast<unsigned int>(w[1]) << 16);
			return static_cast<int>(u) / scale;
		}

		inline void PutBits( unsigned short& w, const bool* bits, int count )
		{
			w = 0;
			for (int i = 0; i < count; i++)
			{
				if (bits[i]) w |= static_cast<unsigned short>(1u << i);
			}
		}
	}

	// the word of the data of the frame to the GPC
	inline constexpr int W_FLAGS = 0;
	inline constexpr int W_SWITCH = 1;
	inline constexpr int W_EE = 2;
	inline constexpr int W_RHC = 3;// 3 words
	inline constexpr int W_THC = 6;// 3 words
	inline constexpr int W_JOINT = 9;// 6 x 2 words
	inline constexpr int W_MASS = 21;// 2 words
	inline constexpr int W_POR = 23;// 3 x 2 words
	inline constexpr int W_AXES = 29;// 9 x 2 words

	/** The MCIU puts the data of the arm in the frame. */
	inline void PackInput( const Input& in, unsigned short frame[FRAME_IN_WORDS] )
	{
		using namespace detail;
		memset( frame, 0, sizeof( unsigned short ) * FRAME_IN_WORDS );

		const bool flags[14] = {in.selected, in.powerOff, in.powerPrimary, in.powerBackup, in.powered, in.backup, in.enter, in.directPlus, in.directMinus,
								in.rateCoarse, in.rateHoldPB, in.autoProceed, in.safeSwitch, in.armFree};
		PutBits( frame[W_FLAGS], flags, 14 );

		frame[W_SWITCH] = static_cast<unsigned short>(((in.modeSwitch + 1) & 0xFF) | (((in.jointSwitch + 1) & 0xFF) << 8));

		const bool ee[8] = {in.eeRigid, in.eeClose, in.eeCapture, in.eeDerigid, in.eeOpen, in.eeExtend, in.payload, in.massEffect};
		PutBits( frame[W_EE], ee, 8 );

		for (int i = 0; i < 3; i++)
		{
			PutS16( &frame[W_RHC + i], in.rhc[i], 6000.0 );// 5 V = 30000
			PutS16( &frame[W_THC + i], in.thc[i], 6000.0 );
		}
		for (int i = 0; i < 6; i++) PutS32( &frame[W_JOINT + (2 * i)], in.joint[i], 10000.0 );// 0.0001 deg
		PutS32( &frame[W_MASS], in.payloadMass, 100.0 );// 0.01 kg
		for (int i = 0; i < 3; i++) PutS32( &frame[W_POR + (2 * i)], in.porOffset.data[i], 100000.0 );// 0.01 mm
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++) PutS32( &frame[W_AXES + (2 * ((3 * i) + j))], in.payloadAxes[i].data[j], 1000000.0 );
		}
	}

	/** The software takes the data of the arm from the frame. */
	inline void UnpackInput( const unsigned short frame[FRAME_IN_WORDS], Input& in )
	{
		using namespace detail;
		in = Input();
		in.dt = GPC_CYCLE;

		const unsigned short f = frame[W_FLAGS];
		in.selected = (f & (1u << 0)) != 0;
		in.powerOff = (f & (1u << 1)) != 0;
		in.powerPrimary = (f & (1u << 2)) != 0;
		in.powerBackup = (f & (1u << 3)) != 0;
		in.powered = (f & (1u << 4)) != 0;
		in.backup = (f & (1u << 5)) != 0;
		in.enter = (f & (1u << 6)) != 0;
		in.directPlus = (f & (1u << 7)) != 0;
		in.directMinus = (f & (1u << 8)) != 0;
		in.rateCoarse = (f & (1u << 9)) != 0;
		in.rateHoldPB = (f & (1u << 10)) != 0;
		in.autoProceed = (f & (1u << 11)) != 0;
		in.safeSwitch = (f & (1u << 12)) != 0;
		in.armFree = (f & (1u << 13)) != 0;

		in.modeSwitch = static_cast<int>(frame[W_SWITCH] & 0xFF) - 1;
		in.jointSwitch = static_cast<int>((frame[W_SWITCH] >> 8) & 0xFF) - 1;

		const unsigned short e = frame[W_EE];
		in.eeRigid = (e & (1u << 0)) != 0;
		in.eeClose = (e & (1u << 1)) != 0;
		in.eeCapture = (e & (1u << 2)) != 0;
		in.eeDerigid = (e & (1u << 3)) != 0;
		in.eeOpen = (e & (1u << 4)) != 0;
		in.eeExtend = (e & (1u << 5)) != 0;
		in.payload = (e & (1u << 6)) != 0;
		in.massEffect = (e & (1u << 7)) != 0;

		for (int i = 0; i < 3; i++)
		{
			in.rhc[i] = GetS16( frame[W_RHC + i], 6000.0 );
			in.thc[i] = GetS16( frame[W_THC + i], 6000.0 );
		}
		for (int i = 0; i < 6; i++) in.joint[i] = GetS32( &frame[W_JOINT + (2 * i)], 10000.0 );
		in.payloadMass = GetS32( &frame[W_MASS], 100.0 );
		for (int i = 0; i < 3; i++) in.porOffset.data[i] = GetS32( &frame[W_POR + (2 * i)], 100000.0 );
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++) in.payloadAxes[i].data[j] = GetS32( &frame[W_AXES + (2 * ((3 * i) + j))], 1000000.0 );
		}
	}

	/** The software puts the commands in the frame. */
	inline void PackOutput( const Output& out, unsigned short frame[FRAME_OUT_WORDS] )
	{
		using namespace detail;
		memset( frame, 0, sizeof( unsigned short ) * FRAME_OUT_WORDS );
		for (int i = 0; i < 6; i++) PutS16( &frame[i], out.rate[i], 1000.0 );// 0.001 deg/s
		const bool flags[5] = {out.safingCancel, out.ready, out.inProg, out.softStop, out.rateMin};
		PutBits( frame[6], flags, 5 );
		frame[7] = static_cast<unsigned short>(out.effMode + 1);
	}

	/** The MCIU takes the commands from the frame. */
	inline void UnpackOutput( const unsigned short frame[FRAME_OUT_WORDS], Output& out )
	{
		using namespace detail;
		for (int i = 0; i < 6; i++) out.rate[i] = GetS16( frame[i], 1000.0 );
		out.safingCancel = (frame[6] & (1u << 0)) != 0;
		out.ready = (frame[6] & (1u << 1)) != 0;
		out.inProg = (frame[6] & (1u << 2)) != 0;
		out.softStop = (frame[6] & (1u << 3)) != 0;
		out.rateMin = (frame[6] & (1u << 4)) != 0;
		out.effMode = static_cast<int>(frame[7]) - 1;
	}

	/**
	 * The MCIU applies the commands of the joint rates to the arm.
	 * The commands are ignored (the arm stops) when the software does not answer; the rates are limited to the mechanical rates of the joints.
	 * @param rate commands of the software [deg/s]
	 * @param valid the software answers
	 * @param joint angles of the joints, updated [deg]
	 */
	inline void ApplyJointRates( const double rate[6], bool valid, double dt, double joint[6] )
	{
		if (!valid) return;
		for (int i = 0; i < 6; i++)
		{
			double r = rate[i];
			if (r > JOINT_MAX_SPEED[i]) r = JOINT_MAX_SPEED[i];
			if (r < -JOINT_MAX_SPEED[i]) r = -JOINT_MAX_SPEED[i];
			double a = joint[i] + (r * dt);
			if (a < rmskin::JOINT_LIMITS[0][i]) a = rmskin::JOINT_LIMITS[0][i];
			if (a > rmskin::JOINT_LIMITS[1][i]) a = rmskin::JOINT_LIMITS[1][i];
			joint[i] = a;
		}
	}
}


#endif// _PDRSPROTOCOL_H_
