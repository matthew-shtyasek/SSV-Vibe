/****************************************************************************
  This file is part of Space Shuttle Vessel

  PDRS software: the logic of the control of the Remote Manipulator System that runs in the GPC (no Orbiter calls, so it can be tested alone)

  In the real vehicle the arm is controlled by software of the GPC: the Manipulator Controller Interface Unit (MCIU) gives the software the
  positions of the hand controllers and the switches of the Displays and Controls panel, and the angles and rates of the joints, and the software
  returns the commands of rate of the joints (and the lights). The software has the modes (single joint, the manual augmented modes ORB UNL, END EFF,
  ORB LD and PL, the automatic modes OPR CMD and AUTO 1 to 4, and TEST), the soft stops, RATE HOLD, the points of resolution (POR) and the displays
  (SPEC 94 PDRS CONTROL, DISP 95 PDRS OVERRIDE, DISP 96 PDRS FAULTS and DISP 169 PDRS STATUS). DIRECT and BACKUP drive, the brakes and the safing
  do not use the computer: they are in the hardware.

  The class is called once per cycle of the software (80 ms in the real software, 12.5 Hz) with the data that the MCIU has sent, and gives the commands
  for the MCIU. Pose of the end effector: forward kinematics of the angles of the joints (RMSKin.h); the motion of the POR is turned into joint
  rates with the inverse kinematics (the new pose of the POR is solved, and the joint rate is the difference of the angles in one cycle).


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
#ifndef _PDRSLOGIC_H_
#define _PDRSLOGIC_H_


#include <OrbiterAPI.h>
#include <EngConst.h>
#include <MathSSV.h>
#include "RMSMath.h"
#include "RMSKin.h"
#include <vector>
#include <string>
#include <sstream>
#include <istream>
#include <cstring>
#include <cmath>
#include <algorithm>


namespace pdrs
{
	inline constexpr double IN2M = 0.0254;
	inline constexpr double FPS2MS = 0.3048;

	// software limits of the rates of the joints of the arm [deg/s], PL ID 0
	inline constexpr double JOINT_MAX_SPEED[6] = {40.0, 40.0, 40.0, 40.0, 40.0, 40.0};// mechanical limits; numbers are made up
	inline constexpr double JOINT_COARSE_SPEED[6] = {1.27, 1.27, 1.78, 2.64, 2.64, 2.64};
	inline constexpr double JOINT_VERN_SPEED[6] = {0.42, 0.42, 0.59, 0.88, 0.88, 0.88};

	// rates of the POR (from the STS-125 PDRS Flight Supplement, PL ID 0)
	inline const double EE_COARSE_ROTATION_SPEED = 4.76 * RAD;// [rad/s]
	inline const double EE_VERN_ROTATION_SPEED = 4.76 * RAD;
	inline constexpr double EE_COARSE_TRANSLATION_SPEED = 2.0 * FPS2MS;// [m/s]
	inline constexpr double EE_VERN_TRANSLATION_SPEED = 0.61 * FPS2MS;

	inline const double Z_AXIS_ANGLE = rmskin::ROLLOUT_ANGLE * RAD;// angle between the Z axis of the EE and the Z axis of the IK frame

	inline constexpr double HC_DETENT = 0.05;// [V] dead band of the hand controllers
	inline constexpr double HC_FULL = 5.0;// [V] full deflection

	/** Indices of the MODE rotary switch positions. */
	enum Mode {MODE_OPR_CMD = 0, MODE_AUTO_1, MODE_AUTO_2, MODE_AUTO_3, MODE_AUTO_4, MODE_ORB_UNL, MODE_END_EFF, MODE_ORB_LD, MODE_PL, MODE_SINGLE, MODE_DIRECT, MODE_TEST};

	enum AutoState {AUTO_IDLE = 0, AUTO_READY, AUTO_RUN};

	/** Data entered in the PDRS CONTROL display (SPEC 94). */
	struct Control
	{
		int plid;// PL ID (item 3): POR used by ORB LD and PL modes (0 = EE tip, unloaded arm)
		int eeid;// END EFF ID (item 4)
		bool io;// I/O ON (item 5) / OFF (item 6)
		bool softstop_ena;// SOFT STOP ENA (item 7) / INH (item 8)
		bool autobrake_ena;// AUTO BRAKE ENA (item 9) / INH (item 10)
		bool posenc_ena;// POS ENC CK ENA (item 11) / INH (item 12)
		bool pohs_ena;// POHS CNTL ENA (item 28) / INH (item 29)
		int autoseq[4];// sequence ID assigned to AUTO 1 to AUTO 4 (items 13 to 16)
		int startpt;// START PT (item 17), 0 = blank
		int lastpt;// LAST PT, 0 = blank
		bool endpos_valid[3];// OPR CMD END POS X, Y, Z entered (items 18 to 20)
		double endpos[3];// [in], in the Orbiter body axes
		bool endatt_valid[3];// OPR CMD END ATT P, Y, R entered (items 21 to 23)
		double endatt[3];// [deg]
		int plinitid;// PL INIT ID (item 24)
		int cmdck;// CMD CK (item 25): 0 = blank, 1 = GOOD, 2 = FAIL
		int wrrange;// WR RANGE (item 26)
		int orbldref;// ORB LD CMD REF (items 31 to 33): 0 = NORM, 1 = FLY-CAMR, 2 = FLY-PL
	};

	/** Data of the PDRS OVERRIDE display (DISP 95): overrides of the switches of panel A8U. */
	struct Override
	{
		bool mode_ena;// MODE SW OVRD (item 1)
		int mode;// mode selected with the items 2 to 12, -1 = none
		bool enter;// ENTER (item 13), one time step
		bool loaded_rate;// LOADED RATE (item 14)
		bool stowed_ops;// STOWED OPS (item 15)
		bool joint_ena;// JNT SW OVRD (item 17)
		int joint;// joint selected with the items 18 to 23, -1 = none
		bool rate_ena;// RATE SW OVRD (item 26)
		bool rate_coarse;// COARSE (item 28), VERNIER (item 27)
		bool auto_ena;// AUTO SW OVRD (item 29)
		bool auto_proceed;// PROCEED (item 30), STOP (item 31)
		bool hc_ena;// HC AXIS CHG (item 32)
		bool safing_cancel;// SAFING CAN (item 35)
	};

	/** Data of the PDRS STATUS display (DISP 169) and PDRS FAULTS display (DISP 96). */
	struct Status
	{
		bool selected;// RMS SELECT in PORT
		int power;// RMS POWER: 0 = OFF, 1 = PRIMARY, 2 = BACKUP
		double jointangle[6];// [deg]
		int reachlimit[6];// -1 or +1 if the joint is beyond a reach limit
		double pos[3];// POR position X, Y, Z (BACS) [in]
		double att[3];// POR attitude P, Y, R [deg]
		double trans_act[3], trans_cmd[3];// translation rates (BACS) [ft/s]
		double rot_act[3], rot_cmd[3];// rotation rates (pitch, yaw, roll) [deg/s]
		bool ee_rigid, ee_close, ee_capture, ee_derigid, ee_open, ee_extend;
	};

	/** Point of an auto sequence. */
	struct AutoPoint
	{
		double pos[3];// POR position X, Y, Z in the Orbiter body axes [in]
		double att[3];// POR attitude P, Y, R [deg]
		bool pause;// programmed pause: the arm comes to rest at this point
	};

	/**
	 * Reads auto sequences. Format: "SEQ n" starts the sequence n (1 to 20); "PT X Y Z P Y R [PAUSE]" adds a point (BACS position in inches,
	 * attitude in degrees); "#" starts a comment.
	 * @return the number of sequences with points
	 */
	inline int ParseAutoSequences( std::istream& file, std::vector<AutoPoint> seq[20] )
	{
		for (int i = 0; i < 20; i++) seq[i].clear();
		std::string line;
		int cur = -1;
		while (std::getline( file, line ))
		{
			size_t comment = line.find( '#' );
			if (comment != std::string::npos) line.erase( comment );

			std::istringstream iss( line );
			std::string key;
			if (!(iss >> key)) continue;

			if (_stricmp( key.c_str(), "SEQ" ) == 0)
			{
				int id;
				if ((iss >> id) && (id >= 1) && (id <= 20)) cur = id - 1;
				else cur = -1;
			}
			else if ((_stricmp( key.c_str(), "PT" ) == 0) && (cur >= 0))
			{
				AutoPoint pt;
				pt.pause = false;
				if (iss >> pt.pos[0] >> pt.pos[1] >> pt.pos[2] >> pt.att[0] >> pt.att[1] >> pt.att[2])
				{
					std::string flag;
					if ((iss >> flag) && (_stricmp( flag.c_str(), "PAUSE" ) == 0)) pt.pause = true;
					if (seq[cur].size() < 200) seq[cur].push_back( pt );
				}
			}
		}
		int count = 0;
		for (int i = 0; i < 20; i++)
		{
			if (!seq[i].empty()) count++;
		}
		return count;
	}

	/** What the MCIU gives the software in a cycle. */
	struct Input
	{
		double dt;// time of the cycle [s]

		// panel A8U and the hand controllers
		bool selected;// RMS SELECT in PORT
		bool powerOff, powerPrimary, powerBackup;// RMS POWER
		bool powered;// the RMS is selected and powered
		bool backup;// RMS POWER in BACKUP only: no computer supported modes
		int modeSwitch;// MODE rotary switch, 0 to 11, -1 if none
		bool enter;// ENTER pushbutton
		int jointSwitch;// joint selected for the single mode, -1 if none
		bool directPlus, directMinus;// DIRECT DRIVE
		double rhc[3], thc[3];// hand controllers [V], 5 V = full deflection (pitch, yaw, roll and X, Y, Z)
		bool rateCoarse;// RATE switch of the RHC in COARSE
		bool rateHoldPB;// RATE HOLD pushbutton of the RHC
		bool autoProceed;// AUTO SEQ switch in PROCEED
		bool safeSwitch;// SAFING switch in SAFE
		bool armFree;// the brakes are released, the arm is not safed and it is not held by a payload attached to something else
		double joint[6];// angles of the joints [deg], with the sign of the shoulder yaw joint inverted (see RMSKin.h)

		// end effector
		bool eeRigid, eeClose, eeCapture, eeDerigid, eeOpen, eeExtend;

		// the payload that the arm holds (the data of the payload that the software has for the PL ID)
		bool payload;
		double payloadMass;// [kg]
		bool massEffect;// the mass of the payload reduces the rates (option of the vessel)
		VECTOR3 porOffset;// vector from the EE tip to the origin of the payload, body axes [m]
		VECTOR3 payloadAxes[3];// axes of the payload, body axes
	};

	/** What the software gives the MCIU in a cycle. */
	struct Output
	{
		double rate[6];// commands of rate of the joints [deg/s]; zero for the joints that are not commanded
		bool safingCancel;// the SAFING CAN item was selected
		int effMode;// operating mode in effect, -1 = none (the MODE light that is lit)
		bool ready, inProg;// lights READY and IN PROG of the AUTO SEQ
		bool softStop;// light SOFT STOP
		bool rateMin;// light RATE MIN (vernier)
	};

	/** Logic of the PDRS software. */
	class Logic
	{
	public:
		Logic( void )
		{
			Reset();
		}

		/** Initial state of the software (as when the software is loaded). */
		void Reset( void )
		{
			ctrl.plid = 0;
			ctrl.eeid = 1;
			ctrl.io = true;// the real software starts with I/O OFF, ON is kept so existing scenarios keep working
			ctrl.softstop_ena = true;
			ctrl.autobrake_ena = true;
			ctrl.posenc_ena = true;
			ctrl.pohs_ena = true;
			for (int i = 0; i < 4; i++) ctrl.autoseq[i] = i + 1;
			ctrl.startpt = 0;
			ctrl.lastpt = 0;
			for (int i = 0; i < 3; i++)
			{
				ctrl.endpos_valid[i] = false;
				ctrl.endpos[i] = 0.0;
				ctrl.endatt_valid[i] = false;
				ctrl.endatt[i] = 0.0;
			}
			ctrl.plinitid = 0;
			ctrl.cmdck = 0;
			ctrl.wrrange = 4;
			ctrl.orbldref = 0;
			ovr.mode_ena = false;
			ovr.mode = -1;
			ovr.enter = false;
			ovr.loaded_rate = false;
			ovr.stowed_ops = false;
			ovr.joint_ena = false;
			ovr.joint = -1;
			ovr.rate_ena = false;
			ovr.rate_coarse = false;
			ovr.auto_ena = false;
			ovr.auto_proceed = false;
			ovr.hc_ena = false;
			ovr.safing_cancel = false;

			effMode = -1;
			acceptedMode = -2;
			lastMode = -1;
			enterPrev = false;
			rateHold = false;
			rateHoldArmed = false;
			rateHoldPBPrev = false;
			for (int i = 0; i < 3; i++) heldRHC[i] = heldTHC[i] = 0.0;
			singleSoftStopIdle = false;
			for (int i = 0; i < 6; i++) singleSoftStopDisabled[i] = false;
			softStop = false;
			autoModeLogged = false;
			autoProceedPrev = false;
			autoState = AUTO_IDLE;
			autoPt = 0;
			ratesValid = false;
			lastPos = _V( 0.0, 0.0, 0.0 );
			lastAtt = _M( 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 );
			transRateAct = rotRateAct = transRateCmd = rotRateCmd = _V( 0.0, 0.0, 0.0 );
			memset( &status, 0, sizeof( status ) );
			for (int i = 0; i < 6; i++) joint[i] = 0.0;
			load = 1.0;
			cycleDt = 0.08;
			ikPos = ikDir = ikRot = _V( 0.0, 0.0, 0.0 );
			in = Input();
			targetValid = false;
		}

		// ---- data of the displays and the state that is saved
		const Control& GetControl( void ) const { return ctrl; }
		Control& GetControl( void ) { return ctrl; }
		const Override& GetOverride( void ) const { return ovr; }
		void GetStatus( Status& st ) const { st = status; }
		int GetAcceptedMode( void ) const { return acceptedMode; }
		void SetAcceptedMode( int mode ) { acceptedMode = mode; }
		int GetEffectiveMode( void ) const { return effMode; }
		int GetAutoState( void ) const { return autoState; }
		bool IsRateHold( void ) const { return rateHold; }
		bool IsSoftStop( void ) const { return softStop; }

		/** Copies the state that the scenarios keep (the data of the displays and the accepted mode) from another logic. */
		void CopyPersistentFrom( const Logic& other )
		{
			ctrl = other.ctrl;
			ovr = other.ovr;
			acceptedMode = other.acceptedMode;
		}

		void SetAutoSequences( const std::vector<AutoPoint> seq[20] )
		{
			for (int i = 0; i < 20; i++) autoseq[i] = seq[i];
		}

		/** Number of points loaded for an auto sequence (ID 1 to 20). */
		int GetAutoSequenceSize( int id ) const
		{
			if ((id < 1) || (id > 20)) return 0;
			return (int)autoseq[id - 1].size();
		}

		/** True if the effective operating mode (selected with MODE and accepted with ENTER) is the given one. */
		bool ModeIs( int mode ) const { return effMode == mode; }

		/** True if one of the 4 manual augmented modes (ORB UNL, END EFF, ORB LD, PL) is selected. */
		bool ManualAugmentedMode( void ) const
		{
			return ModeIs( MODE_ORB_UNL ) || ModeIs( MODE_END_EFF ) || ModeIs( MODE_ORB_LD ) || ModeIs( MODE_PL );
		}

		/** The software cycle. */
		void Step( const Input& input, Output& out )
		{
			in = input;
			cycleDt = in.dt;
			for (int i = 0; i < 6; i++)
			{
				joint[i] = in.joint[i];
				out.rate[i] = 0.0;
				target[i] = joint[i];
			}
			targetValid = false;
			out.safingCancel = false;
			out.effMode = effMode;
			out.ready = out.inProg = out.softStop = out.rateMin = false;

			// the pose of the end effector from the joints
			{
				const rmskin::IKPose ik = rmskin::ForwardIK( joint );
				ikPos = ik.pos;
				ikDir = ik.dir;
				ikRot = ik.rot;
			}
			load = (in.payload) ? RMSMath::LoadScale( in.payloadMass, in.massEffect, ovr.loaded_rate ) : 1.0;

			// the status for the displays (the rates of the POR and the angles are updated with the power on or off)
			UpdateStatus();
			if (in.dt > 0.0) UpdateRates( in.dt );

			if (!in.powered) return;

			// MODE switch position
			int curmode = in.modeSwitch;
			if (ovr.mode_ena && (ovr.mode >= 0)) curmode = ovr.mode;// MODE SW OVRD
			// the mode selected with MODE is entered with ENTER (not needed for DIRECT); if the MODE switch is moved, the software is idle until ENTER is pushed
			const bool enter = in.enter || ovr.enter;
			ovr.enter = false;
			if (acceptedMode == -2) acceptedMode = curmode;// nothing stored in the scenario
			if (enter && !enterPrev) acceptedMode = curmode;
			enterPrev = enter;
			if (curmode == MODE_DIRECT) effMode = MODE_DIRECT;
			else if (curmode == acceptedMode) effMode = curmode;
			else effMode = -1;

			// with RMS POWER in BACKUP, only the BACKUP CONTROL switches work (no computer supported modes)
			if (in.backup) effMode = -1;

			// detect mode changes: RATE HOLD is cancelled and the soft stop logic of SINGLE mode is initialized
			if (effMode != lastMode)
			{
				lastMode = effMode;
				autoState = AUTO_IDLE;
				autoPt = 0;
				rateHold = false;
				singleSoftStopIdle = false;
				autoModeLogged = false;
				// soft stops are disabled for the joints that are already in a soft stop when SINGLE mode is selected
				for (int i = rmskin::SHOULDER_YAW; i <= rmskin::WRIST_ROLL; i++)
				{
					singleSoftStopDisabled[i] = (joint[i] < rmskin::JOINT_SOFTSTOPS[0][i]) || (joint[i] > rmskin::JOINT_SOFTSTOPS[1][i]);
				}
			}

			// SAFING CAN of the PDRS OVERRIDE display is a command for the MCIU (the safing is in the hardware)
			out.safingCancel = ovr.safing_cancel;
			if (!in.safeSwitch) ovr.safing_cancel = false;

			const bool armfree = in.armFree;
			if (!armfree) rateHold = false;

			// RATE HOLD pushbutton (RHC grip): push to hold the commanded rates, push again to release
			if (in.rateHoldPB && !rateHoldPBPrev)
			{
				if (rateHold) rateHold = false;
				else if (armfree && ManualAugmentedMode())
				{
					for (int i = 0; i < 3; i++)
					{
						heldRHC[i] = in.rhc[i];
						heldTHC[i] = in.thc[i];
					}
					rateHoldArmed = false;
					rateHold = true;
				}
			}
			rateHoldPBPrev = in.rateHoldPB;

			transRateCmd = rotRateCmd = _V( 0.0, 0.0, 0.0 );

			if (armfree)
			{
				if (ModeIs( MODE_SINGLE ))
				{
					// joint-by-joint drive
					int dir = 0;
					if (in.directPlus) dir = 1;
					else if (in.directMinus) dir = -1;
					if (dir != 0)
					{
						const int j = SelectedJoint();
						if (j != -1) DriveSingleJoint( j, dir );
					}
				}
				else if (ModeIs( MODE_TEST ))
				{
					// TEST is equivalent to ORB LD, but with zero joint rates: the hand controllers can be checked out and the arm is limp
				}
				else if (ManualAugmentedMode())
				{
					ManualAugmented();
				}
			}

			// OPR CMD, AUTO 1-4
			RunAutoModes( armfree );

			CheckSoftwareStop();
			// the software drops out of mode (idle) when a joint reaches a soft stop, which cancels RATE HOLD
			if (softStop) rateHold = false;

			// the commands of rate of the joints: the angles of the target, in one cycle
			if (targetValid && (in.dt > 0.0))
			{
				for (int i = 0; i < 6; i++) out.rate[i] = (target[i] - joint[i]) / in.dt;
			}

			out.effMode = effMode;
			out.ready = (autoState == AUTO_READY);
			out.inProg = (autoState == AUTO_RUN);
			out.softStop = softStop;
			out.rateMin = !RateCoarse();
		}

		// ---- entries of the displays

		/** Item entry of the PDRS CONTROL display (SPEC 94). @return false if the entry is not valid */
		bool ItemInput( int item, const char* data )
		{
			int n;
			double d;
			const bool nodata = (strlen( data ) == 0);

			switch (item)
			{
				case 3:// PL ID
					if (sscanf_s( data, "%d", &n ) != 1 || (n < 0) || (n > 5)) return false;
					ctrl.plid = n;
					ctrl.cmdck = 0;
					return true;
				case 4:// END EFF ID
					if (sscanf_s( data, "%d", &n ) != 1 || (n < 1) || (n > 2)) return false;
					ctrl.eeid = n;
					return true;
				case 5:
				case 6:// I/O ON/OFF
					if (!nodata) return false;
					ctrl.io = (item == 5);
					return true;
				case 7:
				case 8:// SOFT STOP ENA/INH
					if (!nodata) return false;
					ctrl.softstop_ena = (item == 7);
					return true;
				case 9:
				case 10:// AUTO BRAKE ENA/INH
					if (!nodata) return false;
					ctrl.autobrake_ena = (item == 9);
					return true;
				case 11:
				case 12:// POS ENC CK ENA/INH
					if (!nodata) return false;
					ctrl.posenc_ena = (item == 11);
					return true;
				case 13:
				case 14:
				case 15:
				case 16:// AUTO 1-4 sequence ID
					if (sscanf_s( data, "%d", &n ) != 1 || (n < 1) || (n > 20)) return false;
					ctrl.autoseq[item - 13] = n;
					return true;
				case 17:// START PT
					if (sscanf_s( data, "%d", &n ) != 1 || (n < 1) || (n > 199)) return false;
					ctrl.startpt = n;
					return true;
				case 18:
				case 19:
				case 20:// OPR CMD END POS X, Y, Z
					if (sscanf_s( data, "%lf", &d ) != 1 || (fabs( d ) > 5000.0)) return false;
					ctrl.endpos[item - 18] = d;
					ctrl.endpos_valid[item - 18] = true;
					ctrl.cmdck = 0;
					return true;
				case 21:
				case 23:// OPR CMD END ATT P, R
					if (sscanf_s( data, "%lf", &d ) != 1 || (d < 0.0) || (d >= 360.0)) return false;
					ctrl.endatt[(item == 21) ? 0 : 2] = d;
					ctrl.endatt_valid[(item == 21) ? 0 : 2] = true;
					ctrl.cmdck = 0;
					return true;
				case 22:// OPR CMD END ATT Y
					if (sscanf_s( data, "%lf", &d ) != 1 || (d < 0.0) || (d >= 360.0) || ((d > 90.0) && (d < 270.0))) return false;
					ctrl.endatt[1] = d;
					ctrl.endatt_valid[1] = true;
					ctrl.cmdck = 0;
					return true;
				case 24:// PL INIT ID
					if (sscanf_s( data, "%d", &n ) != 1 || (n < 0) || (n > 5)) return false;
					ctrl.plinitid = n;
					ctrl.cmdck = 0;
					return true;
				case 25:// CMD CK
					if (!nodata) return false;
					ctrl.cmdck = CheckOPRCMD() ? 1 : 2;
					return true;
				case 26:// WR RANGE
					if (sscanf_s( data, "%d", &n ) != 1 || (n < 1) || (n > 6)) return false;
					ctrl.wrrange = n;
					return true;
				case 27:// AUTO BRAKE CK
				case 30:// POR DISPLAY
					return nodata;
				case 28:
				case 29:// POHS CNTL ENA/INH
					if (!nodata) return false;
					ctrl.pohs_ena = (item == 28);
					return true;
				case 31:
				case 32:
				case 33:// ORB LD CMD REF NORM/FLY-CAMR/FLY-PL
					if (!nodata) return false;
					ctrl.orbldref = item - 31;
					return true;
				case 34:// RMS S/N
					return (sscanf_s( data, "%d", &n ) == 1);
				default:
					return false;
			}
		}

		/** Item entry of the PDRS OVERRIDE display (DISP 95). @return false if the entry is not valid */
		bool OverrideItem( int item, const char* data )
		{
			if (strlen( data ) != 0) return false;

			switch (item)
			{
				case 1:// MODE SW OVRD
					ovr.mode_ena = !ovr.mode_ena;
					return true;
				case 2: ovr.mode = MODE_ORB_UNL; return true;
				case 3: ovr.mode = MODE_SINGLE; return true;
				case 4: ovr.mode = MODE_END_EFF; return true;
				case 5: ovr.mode = MODE_ORB_LD; return true;
				case 6: ovr.mode = MODE_PL; return true;
				case 7: ovr.mode = MODE_OPR_CMD; return true;
				case 8: ovr.mode = MODE_AUTO_1; return true;
				case 9: ovr.mode = MODE_AUTO_2; return true;
				case 10: ovr.mode = MODE_AUTO_3; return true;
				case 11: ovr.mode = MODE_AUTO_4; return true;
				case 12: ovr.mode = MODE_TEST; return true;
				case 13:// ENTER
					ovr.enter = true;
					return true;
				case 14:// LOADED RATE
					ovr.loaded_rate = !ovr.loaded_rate;
					return true;
				case 15:// STOWED OPS
					ovr.stowed_ops = !ovr.stowed_ops;
					return true;
				case 16:// REASSIGN
					return true;
				case 17:// JNT SW OVRD
					ovr.joint_ena = !ovr.joint_ena;
					return true;
				case 18:
				case 19:
				case 20:
				case 21:
				case 22:
				case 23:// SH YAW, SH PITCH, EL PITCH, WR PITCH, WR YAW, WR ROLL
					ovr.joint = item - 18;
					return true;
				case 24:
				case 25:// EE TEMP, CRIT TEMP
					return true;
				case 26:// RATE SW OVRD
					ovr.rate_ena = !ovr.rate_ena;
					return true;
				case 27:
					ovr.rate_coarse = false;
					return true;
				case 28:
					ovr.rate_coarse = true;
					return true;
				case 29:// AUTO SW OVRD
					ovr.auto_ena = !ovr.auto_ena;
					return true;
				case 30:
					ovr.auto_proceed = true;
					return true;
				case 31:
					ovr.auto_proceed = false;
					return true;
				case 32:// HC AXIS CHG
					ovr.hc_ena = !ovr.hc_ena;
					return true;
				case 33:
				case 34:
					return true;
				case 35:// SAFING CAN
					ovr.safing_cancel = true;
					return true;
				case 36:
				case 37:
				case 38:// ABE OVRD A, B, C
					return true;
				default:
					return false;
			}
		}

		/** Checks if the OPR CMD end position and attitude can be reached (CMD CK, item 25). */
		bool CheckOPRCMD( void ) const
		{
			for (int i = 0; i < 3; i++)
			{
				if (!ctrl.endpos_valid[i] || !ctrl.endatt_valid[i]) return false;
			}

			VECTOR3 dB, axis;
			double angle;
			GetPORError( ctrl.endpos, ctrl.endatt, dB, axis, angle );

			VECTOR3 porOffset;
			const bool usepor = GetAutoPOR( porOffset );

			// rotation matrix for the whole attitude error
			MATRIX3 R = RMSMath::MatrixFromAxisAngle( axis, angle );

			VECTOR3 newDir = RotateVectorX( mul( R, RotateVectorX( ikDir, -rmskin::ROLLOUT_ANGLE ) ), rmskin::ROLLOUT_ANGLE );
			VECTOR3 newRot = RotateVectorX( mul( R, RotateVectorX( ikRot, -rmskin::ROLLOUT_ANGLE ) ), rmskin::ROLLOUT_ANGLE );
			VECTOR3 newPos = ikPos + RotateVectorX( dB, rmskin::ROLLOUT_ANGLE );
			if (usepor) newPos += RotateVectorX( porOffset - mul( R, porOffset ), rmskin::ROLLOUT_ANGLE );

			double angles[6];
			if (!rmskin::Inverse( newPos, newDir, newRot, joint, angles )) return false;
			for (int i = rmskin::SHOULDER_YAW; i <= rmskin::WRIST_ROLL; i++)
			{
				if ((angles[i] < rmskin::JOINT_REACHLIMITS[0][i]) || (angles[i] > rmskin::JOINT_REACHLIMITS[1][i])) return false;
			}
			return true;
		}

		/** Position (BACS, [in]) and attitude of the EE tip, from the joints of the last cycle. */
		void GetEEState( VECTOR3& pos, MATRIX3& att ) const
		{
			const rmskin::Pose p = rmskin::Forward( joint );
			// position, same calculation as for the LED displays: BACS, in inches
			VECTOR3 ee = (p.posEE - rmskin::SP_JOINT) / IN2M;
			pos = _V( ee.z, ee.x, -ee.y ) + _V( -679.5, -108.0, -444.77 );

			// attitude: reference frame defined by the EE direction, in the Orbiter body axes
			VECTOR3 x = rmskin::ToRMSFrame( p.dirEE );
			VECTOR3 z = rmskin::ToRMSFrame( -p.rotEE );
			VECTOR3 y = crossp( z, x );
			att = _M( x.x, y.x, z.x,
					  x.y, y.y, z.y,
					  x.z, y.z, z.z );
		}

	private:
		Control ctrl;
		Override ovr;
		Input in;
		Status status;

		double joint[6];// angles at the start of the cycle
		double target[6];// angles of the target at the end of the cycle
		bool targetValid;
		double cycleDt;
		double load;// factor of the rates for the payload
		VECTOR3 ikPos, ikDir, ikRot;// pose of the end effector in the frame of the inverse kinematics

		int effMode, acceptedMode, lastMode;
		bool enterPrev;
		bool rateHold, rateHoldArmed, rateHoldPBPrev;
		double heldRHC[3], heldTHC[3];
		bool singleSoftStopIdle;
		bool singleSoftStopDisabled[6];
		bool softStop;
		bool autoModeLogged, autoProceedPrev;
		int autoState;
		int autoPt;
		std::vector<AutoPoint> autoseq[20];

		bool ratesValid;
		VECTOR3 lastPos;
		MATRIX3 lastAtt;
		VECTOR3 transRateAct, rotRateAct;
		VECTOR3 transRateCmd, rotRateCmd;

		bool RateCoarse( void ) const
		{
			// the RATE switch of the RHC, or the override of the PDRS OVERRIDE display
			return ovr.rate_ena ? ovr.rate_coarse : in.rateCoarse;
		}

		int SelectedJoint( void ) const
		{
			if (ovr.joint_ena && (ovr.joint >= 0)) return ovr.joint;// JNT SW OVRD
			return in.jointSwitch;
		}

		bool GetPayloadPOR( VECTOR3& porOffset, VECTOR3 axes[3] ) const
		{
			if (!in.payload) return false;
			porOffset = in.porOffset;
			for (int i = 0; i < 3; i++) axes[i] = in.payloadAxes[i];
			return true;
		}

		bool GetAutoPOR( VECTOR3& porOffset ) const
		{
			porOffset = _V( 0.0, 0.0, 0.0 );
			if (ctrl.plid <= 0) return false;
			VECTOR3 axes[3];
			return GetPayloadPOR( porOffset, axes );
		}

		void GetPORError( const double tgtpos[3], const double tgtatt[3], VECTOR3& dB, VECTOR3& axis, double& angle ) const
		{
			VECTOR3 cur;
			MATRIX3 curM;
			GetEEState( cur, curM );

			VECTOR3 porOffset;
			GetAutoPOR( porOffset );// zero for the EE tip
			RMSMath::PORError( cur, curM, porOffset, tgtpos, tgtatt, dB, axis, angle );
		}

		void UpdateStatus( void )
		{
			status.selected = in.selected;
			status.power = in.powerOff ? 0 : ((in.powerBackup && !in.powerPrimary) ? 2 : 1);
			for (int i = 0; i < 6; i++)
			{
				status.jointangle[i] = joint[i];
				status.reachlimit[i] = 0;
				if (joint[i] < rmskin::JOINT_REACHLIMITS[0][i]) status.reachlimit[i] = -1;
				else if (joint[i] > rmskin::JOINT_REACHLIMITS[1][i]) status.reachlimit[i] = 1;
			}
			status.jointangle[rmskin::SHOULDER_YAW] = -status.jointangle[rmskin::SHOULDER_YAW];// HACK the sign of the shoulder yaw joint is inverted (see RMSKin.h)

			VECTOR3 pos;
			MATRIX3 att;
			GetEEState( pos, att );
			VECTOR3 angles = GetYZX_PYRAnglesFromMatrix( att );
			for (int i = 0; i < 3; i++)
			{
				status.pos[i] = pos.data[i];
				double a = angles.data[i] * DEG;
				if (a < 0.0) a += 360.0;
				status.att[i] = a;
			}

			// rates: the Orbiter body axes (tail, port, down) are converted to BACS (forward, starboard, down); rotations are pitch, yaw and roll
			status.trans_act[0] = -transRateAct.x;
			status.trans_act[1] = -transRateAct.y;
			status.trans_act[2] = transRateAct.z;
			status.trans_cmd[0] = -transRateCmd.x;
			status.trans_cmd[1] = -transRateCmd.y;
			status.trans_cmd[2] = transRateCmd.z;
			status.rot_act[PITCH] = rotRateAct.y;
			status.rot_act[YAW] = rotRateAct.z;
			status.rot_act[ROLL] = rotRateAct.x;
			status.rot_cmd[PITCH] = rotRateCmd.y;
			status.rot_cmd[YAW] = rotRateCmd.z;
			status.rot_cmd[ROLL] = rotRateCmd.x;

			status.ee_rigid = in.eeRigid;
			status.ee_close = in.eeClose;
			status.ee_capture = in.eeCapture;
			status.ee_derigid = in.eeDerigid;
			status.ee_open = in.eeOpen;
			status.ee_extend = in.eeExtend;
		}

		/** Updates the actual rates of the POR (for the PDRS STATUS display). */
		void UpdateRates( double dt )
		{
			VECTOR3 pos;
			MATRIX3 att;
			GetEEState( pos, att );

			if (!ratesValid)
			{
				lastPos = pos;
				lastAtt = att;
				ratesValid = true;
				return;
			}

			// translation rates in BACS, ft/s
			const VECTOR3 vel = (pos - lastPos) / dt / 12.0;
			// rotation rates: the rotation from the last to the current attitude, in the Orbiter body axes
			VECTOR3 axis;
			double angle;
			RMSMath::AxisAngleFromMatrix( mul( att, Transpose( lastAtt ) ), axis, angle );
			const VECTOR3 w = axis * (angle * DEG / dt);// x = roll axis, y = pitch axis, z = yaw axis

			// smoothing, so the display is readable
			const double k = (std::min)( 1.0, dt / 0.25 );
			transRateAct = transRateAct + ((vel - transRateAct) * k);
			rotRateAct = rotRateAct + ((w - rotRateAct) * k);

			lastPos = pos;
			lastAtt = att;
		}

		void CheckSoftwareStop( void )
		{
			softStop = singleSoftStopIdle;
			for (int i = rmskin::SHOULDER_YAW; i <= rmskin::WRIST_ROLL; i++)
			{
				if ((joint[i] < rmskin::JOINT_SOFTSTOPS[0][i]) || (joint[i] > rmskin::JOINT_SOFTSTOPS[1][i]))
				{
					softStop = true;
					return;
				}
			}
		}

		/** Drives one joint in SINGLE mode, dir = +1 or -1. */
		void DriveSingleJoint( int j, int dir )
		{
			// SINGLE drives the joint at a fixed rate, selected by the RATE switch
			const double rate = (RateCoarse() ? JOINT_COARSE_SPEED[j] : JOINT_VERN_SPEED[j]) * load;
			// HACK the sign of the shoulder yaw joint is inverted (see RMSKin.h)
			const double sign = (j == rmskin::SHOULDER_YAW) ? -dir : dir;
			double newangle = joint[j] + (sign * rate * cycleDt);

			// in SINGLE mode the GPC stops the joint at the soft stop and drops to idle, SINGLE must be selected again;
			// soft stops are disabled for a joint that is in a soft stop when SINGLE is selected
			if (singleSoftStopIdle) return;
			if (!singleSoftStopDisabled[j])
			{
				if (newangle < rmskin::JOINT_SOFTSTOPS[0][j])
				{
					newangle = rmskin::JOINT_SOFTSTOPS[0][j];
					singleSoftStopIdle = true;
				}
				else if (newangle > rmskin::JOINT_SOFTSTOPS[1][j])
				{
					newangle = rmskin::JOINT_SOFTSTOPS[1][j];
					singleSoftStopIdle = true;
				}
			}
			SetTarget( j, newangle );
		}

		void SetTarget( int j, double angle )
		{
			target[j] = angle;
			targetValid = true;
		}

		/** Takes the angles of a new pose of the end effector as the target of the cycle. */
		bool MoveEE( const VECTOR3& newPos, const VECTOR3& newDir, const VECTOR3& newRot )
		{
			double a[6];// angles in degrees
			if (!rmskin::Inverse( newPos, newDir, newRot, joint, a )) return false;

			// make sure the speed of each joint is within the limits
			bool move = true;
			for (int i = rmskin::SHOULDER_YAW; i <= rmskin::WRIST_ROLL; i++)
			{
				if (softStop) move = false;
				const double speed = fabs( a[i] - joint[i] ) / cycleDt;
				if (speed > JOINT_MAX_SPEED[i]) move = false;
			}
			if (!move) return false;

			for (int i = rmskin::SHOULDER_YAW; i <= rmskin::WRIST_ROLL; i++)
			{
				// the joint can not go beyond its hard stops
				double angle = a[i];
				if (angle < rmskin::JOINT_LIMITS[0][i]) angle = joint[i];
				if (angle > rmskin::JOINT_LIMITS[1][i]) angle = joint[i];
				SetTarget( i, angle );
			}
			return true;
		}

		void Translate( const VECTOR3& dPos, VECTOR3& newPos )
		{
			if (ModeIs( MODE_END_EFF ))
			{
				// Reference Frame:
				// X: in direction of EE Z: opposite to camera direction Y: completes RH frame
				VECTOR3 y_axis = crossp( ikRot, ikDir );
				// rotation matrix to convert a vector from the EE frame to the Orbiter body frame
				MATRIX3 EERotMatrix = _M( ikDir.x, y_axis.x, ikRot.x,
									   ikDir.y, y_axis.y, ikRot.y,
									   ikDir.z, y_axis.z, ikRot.z );
				// matrix to convert a vector from the EE frame to the IK frame
				MATRIX3 IKRotMatrix = mul( EERotMatrix, Transpose( GetRotationMatrix( _V( 1, 0, 0 ), Z_AXIS_ANGLE ) ) );
				newPos += mul( IKRotMatrix, dPos );
			}
			else if (ModeIs( MODE_ORB_UNL ) || ModeIs( MODE_ORB_LD ) || ModeIs( MODE_PL ))
			{
				VECTOR3 dPosB = dPos;// translation in Orbiter body axes
				VECTOR3 porOffset;
				VECTOR3 axes[3];
				if (ModeIs( MODE_PL ) && (ctrl.plid > 0) && GetPayloadPOR( porOffset, axes ))
				{
					// PL mode translates along the payload axes
					dPosB = (axes[0] * dPos.x) + (axes[1] * dPos.y) + (axes[2] * dPos.z);
				}
				newPos += RotateVectorX( dPosB, rmskin::ROLLOUT_ANGLE );
			}
		}

		void Rotate( const VECTOR3& dAngles, VECTOR3& newPos, VECTOR3& newDir, VECTOR3& newRot )
		{
			if (ModeIs( MODE_END_EFF ))
			{
				// NOTE: EE mode rotates relative to camera orientation
				// we do not need to compensate for angle with RMS and shuttle frames
				// in EE mode, Z-axis is in opposited direction to arm_ee_rot
				VECTOR3 y_axis = crossp( ikRot, ikDir );
				// rotation matrix corresponding to the current orientation
				MATRIX3 RotMatrix = _M( ikDir.x, y_axis.x, ikRot.x,
									   ikDir.y, y_axis.y, ikRot.y,
									   ikDir.z, y_axis.z, ikRot.z );
				// convert the rotation matrix from the IK frame to the EE frame
				RotMatrix = mul( RotMatrix, GetRotationMatrix( _V( 1, 0, 0 ), -Z_AXIS_ANGLE ) );
				// update the rotation matrix to adjust for the new angles
				MATRIX3 RotMatrixRoll, RotMatrixPitch, RotMatrixYaw;
				GetRotMatrixX( dAngles.data[ROLL], RotMatrixRoll );
				GetRotMatrixY( dAngles.data[PITCH], RotMatrixPitch );
				GetRotMatrixZ( dAngles.data[YAW], RotMatrixYaw );
				RotMatrix = mul( RotMatrix, RotMatrixPitch );
				RotMatrix = mul( RotMatrix, RotMatrixYaw );
				RotMatrix = mul( RotMatrix, RotMatrixRoll );
				// convert the rotation matrix from the EE frame to the IK frame
				RotMatrix = mul( RotMatrix, GetRotationMatrix( _V( 1, 0, 0 ), Z_AXIS_ANGLE ) );

				newDir = _V( RotMatrix.m11, RotMatrix.m21, RotMatrix.m31 );
				newRot = _V( RotMatrix.m13, RotMatrix.m23, RotMatrix.m33 );
			}
			else if (ModeIs( MODE_ORB_UNL ) || ModeIs( MODE_ORB_LD ) || ModeIs( MODE_PL ))
			{
				// ORB UNL rotates about the EE tip; ORB LD and PL rotate about the POR of the payload selected with PL ID
				// (the Orbiter body axes for ORB LD, the payload axes for PL)
				VECTOR3 porOffset = _V( 0.0, 0.0, 0.0 );
				VECTOR3 axes[3];
				bool payload = false;
				if ((ModeIs( MODE_ORB_LD ) || ModeIs( MODE_PL )) && (ctrl.plid > 0)) payload = GetPayloadPOR( porOffset, axes );
				if (!payload || !ModeIs( MODE_PL ))
				{
					axes[0] = _V( 1.0, 0.0, 0.0 );
					axes[1] = _V( 0.0, 1.0, 0.0 );
					axes[2] = _V( 0.0, 0.0, 1.0 );
				}
				RotateAboutPOR( _V( dAngles.data[ROLL], dAngles.data[PITCH], dAngles.data[YAW] ), axes, payload, porOffset, newPos, newDir, newRot );
			}
		}

		void RotateAboutPOR( const VECTOR3& angles, const VECTOR3 axes[3], bool usePOR, const VECTOR3& porOffset, VECTOR3& newPos, VECTOR3& newDir, VECTOR3& newRot )
		{
			RMSMath::RotateAboutPOR( angles, axes, usePOR, porOffset, rmskin::ROLLOUT_ANGLE, ikDir, ikRot, newPos, newDir, newRot );
		}

		/** Manual augmented modes: the hand controllers give the rates of the POR. */
		void ManualAugmented( void )
		{
			VECTOR3 newPos = ikPos;
			VECTOR3 newDir = ikDir;
			VECTOR3 newRot = ikRot;
			bool rotateEE = false, translateEE = false;
			const double loadscale = load;
			const double simdt = cycleDt;

			// hand controller commands (volts, 5V = full deflection); the rates are proportional to the deflection
			double rhc[3], thc[3];
			bool detent = true;
			for (int i = 0; i < 3; i++)
			{
				rhc[i] = in.rhc[i];
				thc[i] = in.thc[i];
				if (!Eq( rhc[i], 0.0, HC_DETENT ) || !Eq( thc[i], 0.0, HC_DETENT )) detent = false;
			}
			if (rateHold)
			{
				if (detent) rateHoldArmed = true;
				for (int i = 0; i < 3; i++)
				{
					// after the hand controllers return to detent, further deflection biases the held rates
					const double rhc_bias = rateHoldArmed ? rhc[i] : 0.0;
					const double thc_bias = rateHoldArmed ? thc[i] : 0.0;
					rhc[i] = (std::max)( -5.0, (std::min)( 5.0, heldRHC[i] + rhc_bias ) );
					thc[i] = (std::max)( -5.0, (std::min)( 5.0, heldTHC[i] + thc_bias ) );
				}
			}

			// EE rotation
			VECTOR3 change = _V( 0.0, 0.0, 0.0 );
			for (int i = 0; i < 3; i++)
			{
				if (!Eq( rhc[i], 0.0, HC_DETENT ))
				{
					if (RateCoarse()) change.data[i] += (rhc[i] / HC_FULL) * simdt * EE_COARSE_ROTATION_SPEED * loadscale;
					else change.data[i] += (rhc[i] / HC_FULL) * simdt * EE_VERN_ROTATION_SPEED * loadscale;
					rotateEE = true;
				}
			}
			if (rotateEE)
			{
				Rotate( change, newPos, newDir, newRot );
				if (!ModeIs( MODE_END_EFF )) rotRateCmd = _V( change.data[ROLL], change.data[PITCH], change.data[YAW] ) * (DEG / simdt);
			}

			// EE translation
			change = _V( 0.0, 0.0, 0.0 );
			for (int i = 0; i < 3; i++)
			{
				if (!Eq( thc[i], 0.0, HC_DETENT ))
				{
					if (RateCoarse()) change.data[i] += (thc[i] / HC_FULL) * simdt * EE_COARSE_TRANSLATION_SPEED * loadscale;
					else change.data[i] += (thc[i] / HC_FULL) * simdt * EE_VERN_TRANSLATION_SPEED * loadscale;
					translateEE = true;
				}
			}
			if (translateEE)
			{
				Translate( change, newPos );
				if (!ModeIs( MODE_END_EFF )) transRateCmd = change / (simdt * FPS2MS);
			}

			if (translateEE || rotateEE) MoveEE( newPos, newDir, newRot );
		}

		/** Gets the target point of the active auto mode. @return false if there is none */
		bool GetAutoTarget( double tgtpos[3], double tgtatt[3], bool& pause ) const
		{
			pause = false;
			if (ModeIs( MODE_OPR_CMD ))
			{
				for (int i = 0; i < 3; i++)
				{
					tgtpos[i] = ctrl.endpos[i];
					tgtatt[i] = ctrl.endatt[i];
				}
				return true;
			}

			const int idx = effMode - MODE_AUTO_1;
			if ((idx < 0) || (idx > 3)) return false;
			const std::vector<AutoPoint>& seq = autoseq[ctrl.autoseq[idx] - 1];
			const int next = autoPt + 1;// the points are numbered from 1
			if ((next < 1) || (next > (int)seq.size())) return false;
			const AutoPoint& pt = seq[next - 1];
			for (int i = 0; i < 3; i++)
			{
				tgtpos[i] = pt.pos[i];
				tgtatt[i] = pt.att[i];
			}
			pause = pt.pause;
			return true;
		}

		/** Runs the OPR CMD and AUTO 1-4 modes. */
		void RunAutoModes( bool armfree )
		{
			const double simdt = cycleDt;
			const bool automode = ModeIs( MODE_OPR_CMD ) || ModeIs( MODE_AUTO_1 ) || ModeIs( MODE_AUTO_2 ) || ModeIs( MODE_AUTO_3 ) || ModeIs( MODE_AUTO_4 );
			const bool proceed = ovr.auto_ena ? ovr.auto_proceed : in.autoProceed;

			if (!automode)
			{
				autoState = AUTO_IDLE;
				autoProceedPrev = proceed;
				return;
			}

			if (!autoModeLogged)
			{
				// mode just entered
				autoModeLogged = true;
				autoState = AUTO_IDLE;
				if (!ModeIs( MODE_OPR_CMD ))
				{
					// AUTO 1-4: the arm must be at the start point of the sequence selected with the items 13-16 of the PDRS CONTROL display
					const int idx = effMode - MODE_AUTO_1;
					const std::vector<AutoPoint>& seq = autoseq[ctrl.autoseq[idx] - 1];
					const int start = (ctrl.startpt > 0) ? ctrl.startpt : 1;
					if (((int)seq.size() >= 2) && (start < (int)seq.size()))
					{
						autoPt = start;
						ctrl.lastpt = start;
						autoState = AUTO_READY;
					}
				}
			}

			if (ModeIs( MODE_OPR_CMD ) && (autoState != AUTO_RUN))
			{
				// OPR CMD is ready when the command check (item 25) is GOOD and PL INIT ID (item 24) matches PL ID (item 3)
				autoState = ((ctrl.cmdck == 1) && (ctrl.plinitid == ctrl.plid)) ? AUTO_READY : AUTO_IDLE;
			}

			// AUTO SEQ switch: PROCEED starts or continues the sequence, STOP brings the arm to rest
			if (proceed && !autoProceedPrev && (autoState == AUTO_READY) && armfree) autoState = AUTO_RUN;
			else if (!proceed && (autoState == AUTO_RUN)) autoState = AUTO_READY;
			autoProceedPrev = proceed;

			if (autoState != AUTO_RUN) return;
			if (!armfree || softStop)
			{
				autoState = AUTO_READY;
				return;
			}

			double tgtpos[3], tgtatt[3];
			bool pause;
			if (!GetAutoTarget( tgtpos, tgtatt, pause ))
			{
				autoState = AUTO_IDLE;
				return;
			}

			VECTOR3 dB, axis;
			double angle;
			GetPORError( tgtpos, tgtatt, dB, axis, angle );
			const double dist = length( dB );

			if ((dist < 0.003) && (angle < (0.1 * RAD)))
			{
				// target point reached
				if (ModeIs( MODE_OPR_CMD ))
				{
					ctrl.cmdck = 0;
					autoState = AUTO_IDLE;
				}
				else
				{
					const int idx = effMode - MODE_AUTO_1;
					const int size = (int)autoseq[ctrl.autoseq[idx] - 1].size();
					autoPt++;
					ctrl.lastpt = autoPt;
					if (autoPt >= size) autoState = AUTO_IDLE;// end of the sequence
					else if (pause) autoState = AUTO_READY;// programmed pause, PROCEED must be selected again
				}
				return;
			}

			const double vt = (RateCoarse() ? EE_COARSE_TRANSLATION_SPEED : EE_VERN_TRANSLATION_SPEED) * load;
			const double vr = (RateCoarse() ? EE_COARSE_ROTATION_SPEED : EE_VERN_ROTATION_SPEED) * load;

			VECTOR3 newPos = ikPos;
			VECTOR3 newDir = ikDir;
			VECTOR3 newRot = ikRot;

			if (angle > 1e-6)
			{
				VECTOR3 porOffset;
				const bool usepor = GetAutoPOR( porOffset );
				const VECTOR3 axes[3] = {_V( 1.0, 0.0, 0.0 ), _V( 0.0, 1.0, 0.0 ), _V( 0.0, 0.0, 1.0 )};
				const double w = (std::min)( vr, (std::max)( angle, 0.002 ) );
				const double step = (std::min)( angle, w * simdt );
				RotateAboutPOR( axis * step, axes, usepor, porOffset, newPos, newDir, newRot );
				rotRateCmd = axis * (step * DEG / simdt);
			}
			if (dist > 1e-6)
			{
				const double v = (std::min)( vt, (std::max)( dist, 0.002 ) );
				const double step = (std::min)( dist, v * simdt );
				newPos += RotateVectorX( (dB / dist) * step, rmskin::ROLLOUT_ANGLE );
				transRateCmd = (dB / dist) * (step / (simdt * FPS2MS));
			}

			if (!MoveEE( newPos, newDir, newRot )) autoState = AUTO_READY;// the arm can not reach the next position
		}
	};
}


#endif// _PDRSLOGIC_H_
