/****************************************************************************
  This file is part of Space Shuttle Vessel

  Universal Pointing Processor principal function


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

#pragma once

#include "../SimpleGPCSoftware.h"

namespace dps
{
	class GNCUtilities;
	class OrbitDAP;

	class UniversalPointing : public SimpleGPCSoftware
	{
		int START_TIME[4]; //V93W6801C-6804C, Maneuver start time, D/H/M/S
		float ROLLI; //V93H6837C, Input roll angle, rad
		float PITCHI; //V93H6838C, Input pitch angle, rad
		float YAWI; //V93H6839C, Input yaw angle, rad
		int TGT_ID; //V93U6810C, Target vector ID (1 = Orbital object, 2 = Center of Earth, 3 = Earth relative target, 4 = Sun, 5 = Celestial, 6-10, Unassigned, 11-110 = Navigation Stars)
		float TGT_RA; //V93H6813C, Target vector right ascension, rad
		float TGT_DEC; //V93H6815C, Target vector declination, rad
		float TGT_LAT; //V93H6820C, Earth-fixed target latitude, rad
		float TGT_LON; //V93H6821C, Earth-fixed target longitude, rad
		float TGT_ALT; //V93H6822C, Earth-fixed target altitude, nmi
		int BODV_ID; //V93U6805C, Body vector identifier (1 = +X axis, 2 = -X axis, 3 = -Z axis, 4 = -Y ST, 5 = Universal Option)
		float BODV_PITCH; //V93U6806C, Body vector pitch coordinate, rad
		float BODV_YAW; //V93U6806C, Body vector yaw coordinate, rad
		float OMICRON; //V93H6825C, Constraint angle about body vector, rad
		int OPT_SEL; //V93J6829C, Selected attitude option
		bool OPT_CNCL; //V93X6871X, Cancel option discrete (0 = no option, 1 = MNVR option, 2 = Tracking option, 3 = Rotation option)
		int MON_AXIS; //V93U6868C, Selected monitor axis (1 = +X control axis, 2 = -X control axis)
		bool ERR_SEL; //V93X6845X, Total/DAP attitude error discrete (false = DAP errors, true = total errors)
		VECTOR3 ATT_ERR; //V95H7490C-7492C, roll, pitch and yaw attitude errors, deg
		VECTOR3 ATT_RATE; //V95H7476C, V95H7477C, V95H7487C, IMU body rate around x/y/z-axis, deg/s

		float ROLLC; //V95H7473C, Current roll angle (YZX Euler), rad
		float PITCHC; //V95H7474C, Current pitch angle (YZX Euler), rad
		float YAWC; //V95H7475C, Current yaw angle (YZX Euler), rad
		float ROLLR; //V95H7467C, Required roll angle (YZX Euler), rad
		float PITCHR; //V95H7468C, Required pitch angle (YZX Euler), rad
		float YAWR; //V95H7484C, Required yaw angle (YZX Euler), rad
		VECTOR3 TOT_ERR; //Total attitude error, deg

		bool THREE_AXIS; //V93X6881X, Three-axis tracking flag
		bool TRK_DCD; //V93X6882X, Tracking data change discrete
		bool ROT_DCD; //V93X6883X, Rotation data change discrete
		bool MNV_DCD; //V93X6884X, Maneuver data change discrete
		bool FUT_DCD; //V93X6885X, Future option data change discrete
		bool PARAM_XFR; //Parameter transfer flag
		int OPT_PROC; //V97U5061C, Option process indicator
		bool ROT_IC; //V96U9754C, Rotation option initial condition indicator
		bool MNVR_TIME_IC; //Maneuver time initial condition indicator
		bool MNVR_TIME_FLAG; //Maneuver time flag
		VECTOR3 PLOS; //Body vector
		VECTOR3 TGPOS; //Earth-fixed target position vector

		//Temporary variables, updated before UNIV_SEQ

		//Current body quaternion
		double Q_BOD_M50_S;
		VECTOR3 Q_BOD_M50_V;
		double GMTC; //Time tag of current body quaternion
		VECTOR3 RORB; //Position vector of Orbiter
		VECTOR3 VORB; //Velocity vector of Orbiter
		VECTOR3 RREL; //Orbiter/target relative position vector
		VECTOR3 VREL; //Orbiter/target relative velocity vector

		//Temporary variables within UNIV_SEQ, don't need to be saved/loaded
		VECTOR3 RR_BOD; //Body roll reference vector
		VECTOR3 RR_M50; //Inertial roll reference vector
		VECTOR3 RRA_M50; //Alternate inertial roll reference vector
		VECTOR3 TPOS; //Earth-fixed target position vector expressed in M50 coordinates
		VECTOR3 TLOS; //Target line-of-sight

		//Currently being used
		VECTOR3 PLOS_C;
		int TGT_ID_C;
		VECTOR3 TGPOS_C;
		double OMICRON_C;
		bool THREE_AXIS_C;
		double Q_MNVRC_S;
		VECTOR3 Q_MNVRC_V;
		double ROTR_RATE_C;
		double Q_BODC_M50_S;
		VECTOR3 Q_BODC_M50_V;
		double GMTS_C;

		int OPT_CUR; //V95J7482C, Current option indicator
		int OPT_FUT; //V95J7483C, Future option indicator
		double MNVR_CMPL_TIME; //V95W7472C, Current maneuver completion time, sec
		double GMTS; //Attitude option start time

		float BODV_SCL[5][2]; //V96U9715C-9724C, Body vector elements

		double GMT; //Current time

		//Maneuver option quaternion
		double Q_MNVR_S;
		VECTOR3 Q_MNVR_V;

		//Display values that are set to be flashing
		bool RA_flash;
		bool DEC_flash;
		bool LAT_flash;
		bool LON_flash;
		bool ALT_flash;
		bool P_flash;
		bool Y_flash;
		bool OMICRON_flash;

		bool PY_executed;
		bool RA_DEC_executed;

		bool AUTO_ALERT;

		//Star table
		VECTOR3 I_STAR_SEL_5; //Celestial target

		//INTERFACE TO ORBIT DAP
		double Q_RBOD_M50_S; //V95U2209C, M50 to body quaternion element 1
		VECTOR3 Q_RBOD_M50_V; //V95U2210C-2212C, M50 to body quaternion element 2-4
		VECTOR3 REQD_BRATE; //Required body attitude rate vector
		double GMTR; //GMT of required body quaternion
		bool MNVR_TRACK_OPTION; // V95X2222X, Maneuver/tracking option discrete

		double lastUpdateSimTime;

		GNCUtilities* pGNCUtilities;
		OrbitDAP* pOrbitDAP;
	public:
		explicit UniversalPointing(SimpleGPCSystem* _gpc);
		virtual ~UniversalPointing();

		void Realize() override;

		void OnPreStep(double simt, double simdt, double mjd) override;

		bool OnMajorModeChange(unsigned int newMajorMode) override;
		bool ItemInput(int item, const char* Data);
		void OnPaint(vc::MDU* pMDU) const;

		bool OnParseLine(const char* keyword, const char* value) override;
		void OnSaveState(FILEHANDLE scn) const override;

		bool Get_MNVR_TRACK_OPTION() const;
		void GetRequiredQuaternion(double& QS, VECTOR3& QV) const;
		VECTOR3 Get_REQD_BRATE() const;
		double Get_GMTR() const;

	private:
		void KLoad();

		void UNIV_SEQ(); //Universal pointing processing sequencing task
		void PARAM_PROC(); //Parameter processing task
		void MNVR_QUAT(); //Maneuver option quaternion subtask
		void STAR(); //Celestial target vector subtask
		void RA_DEC(VECTOR3& TGT_VEC); //Celestial target RA/DEC subtask
		void BODY_VEC(); //Body vector subtask
		void PY(); //Body vector PITC/YAW subtask
		void ETGT_POS(); //Earth-fixed target position vector subtask
		void CYC_LVLH(); //Cyclic LVLH mode task
		void CYC_OPT(); //Cyclic attitude option computations task
		void MNVR(); //Universal pointing maneuver task
		void TRACK(); //Universal pointing processor tracking task
		void LOS_VEC(); //Target line-of-sight vector subtask
		void F2AXIS(VECTOR3 P, VECTOR3 T); //Two-axis tracking subtask
		void F3AXIS(VECTOR3 VEC_BOD, VECTOR3 VEC_M50, double ROLL, double& Q_ATT_M50_S, VECTOR3& Q_ATT_M50_V); //Three-axis tracking subtask
		void TRK_RATE(); //Tracking rate vector subtask
		void ROTR(); //Universal pointing rotation task
		void DISP_SUP(); //Display support task
		void ATT_ANG(double QS, VECTOR3 QV, float& ROLL, float& PITCH, float& YAW) const; //Attitude angles subtask
		void ATT_ERROR(); //Attitude error subtask
		void CMPLT_TIME(); //Maneuver completion time subtask
		void RNDZTGT(); //Rendezvous target task
	};
}
