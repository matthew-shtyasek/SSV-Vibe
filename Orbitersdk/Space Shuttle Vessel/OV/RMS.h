/****************************************************************************
  This file is part of Space Shuttle Vessel

  Remote Manipulator System subsystem definition


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
/******* SSV File Modification Notice *******
Date         Developer
2020/04/01   GLS
2020/04/07   GLS
2020/05/06   GLS
2020/05/10   GLS
2020/05/14   GLS
2020/06/20   GLS
2020/10/10   GLS
2021/06/28   GLS
2021/07/02   GLS
2021/07/03   GLS
2021/08/23   GLS
2021/08/24   GLS
2021/09/20   GLS
2022/01/02   GLS
2022/01/03   GLS
2022/01/04   GLS
2022/01/10   GLS
2022/01/12   GLS
2022/02/16   GLS
2022/03/24   GLS
2022/03/26   GLS
2022/08/05   GLS
2022/09/29   GLS
2022/10/30   GLS
2022/11/01   GLS
2022/11/09   GLS
2022/11/12   GLS
2023/01/15   GLS
2023/02/12   GLS
2023/02/13   GLS
2023/03/26   GLS
********************************************/
/****************************************************************************
  This file is part of Space Shuttle Ultra

  Remote Manipulator System definition



  Space Shuttle Ultra is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  Space Shuttle Ultra is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Space Shuttle Ultra; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

  See https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html or
  file Doc\Space Shuttle Ultra\GPL.txt for more details.

  **************************************************************************/
#ifndef __RMS_H
#define __RMS_H


#include "RMSReaction.h"
#include "PDRSProtocol.h"
#include "MassProps.h"
#include "MPM.h"
#include <vector>
#include "mission/Mission.h"


class ExternalLight;
class CCTVCamera;
class CCTVCameraPTU;
class RemoteVideoSwitcher;


class RMS : public MPM
{
public:
	typedef enum {SHOULDER_YAW=0, SHOULDER_PITCH=1, ELBOW_PITCH=2, WRIST_PITCH=3, WRIST_YAW=4, WRIST_ROLL=5} RMS_JOINT;

	RMS( AtlantisSubsystemDirector* _director, const std::string& _ident, bool portside, const mission::MissionRMS& missionrms );
	virtual ~RMS();

	void Realize() override;
	void OnPreStep(double simt, double simdt, double mjd) override;
	void OnPostStep(double simt, double simdt, double mjd) override;
	bool OnParseLine(const char* line) override;
	void OnSaveState(FILEHANDLE scn) const override;
	void VisualCreated( VISHANDLE vis ) override;
	bool SingleParamParseLine() const override {return true;};

	// mass value from Shuttle Systems Weight & Performance Monthly Status Report, Dec. 30 1983 (NASA-TM-85494)
	double GetSubsystemMass() const override {return 426.8304;};

	void CreateAttachment() override;
	virtual void ShiftCG( const VECTOR3& shift ) override;

	void UpdateAttachment( void );

	/**
	 * Returns true if arm is free to move.
	 * Returns false if arm is grappled to payload which is attached to something else.
	 */
	bool Movable() const;

	/**
	 * The software of the GPC that runs the computer supported modes registers its logic here. The arm sends the data of the MCIU
	 * to the software and gets the commands of rate of the joints through the mailbox (the bus).
	 */
	void SetGpcLogic( pdrs::Logic* logic );
	pdrs::Logic* GetGpcLogic( void ) const {return gpcLogic;}
	pdrs::Mailbox& GetMailbox( void ) {return mailbox;}

	/** @return true if the computer supported modes are run by the GPC (option RMSViaGPC and the software is loaded) */
	bool UsingGpc( void ) const;

	/**
	 * Data entered in the PDRS CONTROL display (SPEC 94).
	 */
	typedef pdrs::Control PDRSControl;
	const PDRSControl& GetPDRSControl() const {return UsingGpc() ? gpcLogic->GetControl() : pdrs;}

	/**
	 * Data of the PDRS OVERRIDE display (DISP 95): overrides of the switches of panel A8U.
	 */
	typedef pdrs::Override PDRSOverride;
	const PDRSOverride& GetPDRSOverride() const {return UsingGpc() ? gpcLogic->GetOverride() : ovr;}

	/**
	 * Handles an item entry in the PDRS OVERRIDE display (DISP 95).
	 * @return false if the entry is not valid
	 */
	bool PDRSOverrideItem( int item, const char* data );

	/**
	 * Data of the PDRS STATUS display (DISP 169) and PDRS FAULTS display (DISP 96).
	 */
	typedef pdrs::Status PDRSStatus;
	void GetPDRSStatus( PDRSStatus& st ) const;

	/**
	 * Gets the distance [m] and the angle error [rad] to the closest grapple fixture, if the EE is empty.
	 * @return false if there is none
	 */
	bool GetGrappleHint( double& distance, double& angle ) const;

	/**
	 * Gets the points of mass of the arm and of the payload that it holds, for the center of mass and the inertia of the orbiter (positions in the frame of the mesh).
	 * @param[out] arm the arm where it is
	 * @param[out] armStowed the arm where it is in the default center of mass of the orbiter (negative mass)
	 * @param[out] payload the payload held by the arm (zero mass if there is none)
	 */
	void GetMassPoints( massprops::Point& arm, massprops::Point& armStowed, massprops::Point& payload ) const;

	/** @return the attachment of the end effector (the payload that the arm holds is attached to it) */
	ATTACHMENTHANDLE GetEEAttachment( void ) const { return hAttach; }

	/**
	 * Handles an item entry in the PDRS CONTROL display (SPEC 94).
	 * @param item item number
	 * @param data entered data (empty for an EXEC without data)
	 * @return false if the entry is not valid
	 */
	bool PDRSItemInput( int item, const char* data );

	/**
	 * Number of points loaded for an auto sequence (ID 1 to 20).
	 */
	int GetAutoSequenceSize( int id ) const;

	/**
	 * Writes the current POR position and attitude to the log, in the format of the auto sequence file (Config\SSV\RMS\AutoSequences.txt).
	 */
	void LogPOR( void ) const;

protected:
	void OnMRLLatched( void ) override;
	void OnMRLReleased( void ) override;

	void OnAttach() override;
	void OnDetach() override;
private:
	void AddMesh( void );
	void DefineAnimations( void );
	bool ArmStowed() const;

	/**
	 * Calculates new EE position (in IK frame) for given delta pos
	 * @param dPos change in position (in meters) in selected RMS control frame
	 * @param newPos new EE position in IK frame
	 */
	void Translate(const VECTOR3 &dPos, VECTOR3& newPos);
	/**
	 * Calculates new EE rotation parameters (in IK frame) for given Euler angles.
	 * @param dAngles YZX Euler angles (around RMS control frame) in radians
	 * @param newPos EE position in IK frame (in/out), changes when the rotation is about a POR different from the EE tip
	 * @param newDir new direction vector of EE after rotation in IK frame
	 * @param newRot new rot vector of EE after rotation in IK frame
	 */
	void Rotate(const VECTOR3 &dAngles, VECTOR3& newPos, VECTOR3& newDir, VECTOR3& newRot); // angles in radians
	bool MoveEE(const VECTOR3 &newPos, const VECTOR3 &newDir, const VECTOR3 &newRot, double simdt);
	void SetJointAngle(RMS_JOINT joint, double angle); //angle in degrees
	/**
	 * Converts vector from Orbitersim frame to RMS frame
	 * Does not rotate vector to correct for RMS rollout angle
	 */
	VECTOR3 ConvertVectorToRMSFrame(const VECTOR3& v) const
	{
		return _V(-v.z, -v.x, -v.y);
	}

	int GetSelectedJoint() const;

	/** Indices of the MODE rotary switch positions (RMS_MODE bundle). */
	enum RMS_MODE {MODE_OPR_CMD = 0, MODE_AUTO_1, MODE_AUTO_2, MODE_AUTO_3, MODE_AUTO_4, MODE_ORB_UNL, MODE_END_EFF, MODE_ORB_LD, MODE_PL, MODE_SINGLE, MODE_DIRECT, MODE_TEST};

	/** True if one of the 4 manual augmented modes (ORB UNL, END EFF, ORB LD, PL) is selected. */
	bool ManualAugmentedMode() const;

	/**
	 * Gets the Point Of Resolution (POR) of a captured payload.
	 * The POR is the payload origin (c.g.), like PL ID 1 in the real system. With no payload captured the POR is the EE tip (PL ID 0).
	 * @param porOffset vector from the EE tip to the POR, in Orbiter body axes
	 * @param axes unit vectors of the payload axes, in Orbiter body axes (the identity if the payload has the orientation of the Orbiter)
	 * @return false if no payload is captured
	 */
	bool GetPayloadPOR( VECTOR3& porOffset, VECTOR3 axes[3] ) const;

	/** Drives one joint in SINGLE or DIRECT mode, dir = +1 or -1. */
	void DriveSingleJoint( int joint, int dir, double simdt );

	/** RMS selected and powered (RMS POWER in PRIMARY or BACKUP). */
	bool Powered( void ) const;

	/** True if the coarse rates are selected (RATE switch of the RHC, or the override). */
	bool RateCoarse( void ) const;

	/** Factor for the rates of a loaded arm, 1 for the unloaded arm. */
	double GetLoadScale( void ) const;

	/** Updates the actual rates of the POR (for the PDRS STATUS display). */
	void UpdateRates( double dt );

	/** True if the effective operating mode (selected with MODE and accepted with ENTER) is the given one. */
	bool ModeIs( int mode ) const {return effMode == mode;}

	typedef pdrs::AutoPoint AutoPoint;
	enum AUTO_STATE {AUTO_IDLE = 0, AUTO_READY, AUTO_RUN};

	/** Reads the auto sequences from Config\SSV\RMS\AutoSequences.txt. */
	void LoadAutoSequences( void );

	/** Gets the position (BACS, [in]) and attitude (RACS, matrix) of the EE tip. */
	void GetEEState( VECTOR3& pos, MATRIX3& att ) const;

	/**
	 * Gets the POR used by the auto modes (the one of the payload for PL ID > 0 and a captured payload, the EE tip otherwise), as the vector from the EE tip.
	 * @return true if the POR is not the EE tip
	 */
	bool GetAutoPOR( VECTOR3& porOffset ) const;

	/**
	 * Calculates the movement from the current POR state to the target POR state.
	 * @param tgtpos target position [in]
	 * @param tgtatt target attitude P, Y, R [deg]
	 * @param dB position error in Orbiter body axes [m]
	 * @param axis axis of the rotation error (Orbiter body axes)
	 * @param angle rotation error [rad]
	 */
	void GetPORError( const double tgtpos[3], const double tgtatt[3], VECTOR3& dB, VECTOR3& axis, double& angle ) const;

	/** Checks if the OPR CMD end position and attitude can be reached (CMD CK, item 25). */
	bool CheckOPRCMD( void ) const;

	/** Gets the target point of the active auto mode. @return false if there is none */
	bool GetAutoTarget( double tgtpos[3], double tgtatt[3], bool& pause ) const;

	/** Runs the OPR CMD and AUTO 1-4 modes. */
	void RunAutoModes( double simdt, bool armfree );

	/** Rotates about the POR (axes in Orbiter body axes), see Rotate() */
	void RotateAboutPOR( const VECTOR3& angles, const VECTOR3 axes[3], bool usePOR, const VECTOR3& porOffset, VECTOR3& newPos, VECTOR3& newDir, VECTOR3& newRot );

	/** Calculates the joint angles for the given EE position and orientation, without moving the arm. */
	bool ComputeJointAngles( const VECTOR3 &newPos, const VECTOR3 &newDir, const VECTOR3 &newRot, double new_joint_angles[6] ) const;

	/**
	 * Updates the EE spotlight position/direction. To be called when the RMS moves and also from the Atlantis c.g. change member.
	 */
	void UpdateEELight( void );

	void AutoGrappleSequence();
	void AutoReleaseSequence();

	/**
	 * Checks if any joint is past its software stop.
	 */
	void CheckSoftwareStop( void );

	void CheckRFL( void );

	/**
	 * Calculates EE, CCTV and light positions, directions and orientations from joint angles.
	 */
	void CalcVectors( void );

	void CreateCCTV( void );

	MESHHANDLE hMesh_RMS;
	MESHHANDLE hMesh_Pedestal;
	UINT mesh_index_RMS;
	UINT mesh_index_pedestal_forward;
	UINT mesh_index_pedestal_mid;
	UINT mesh_index_pedestal_aft;

	DiscInPort RMSSelect;

	UINT anim_joint[6], anim_rms_ee;

	CCTVCameraPTU* cameraElbow;
	CCTVCamera* cameraWrist;
	RemoteVideoSwitcher* videoswitcher;


	ExternalLight* light;

	//EE and IK parameters
	/** Refence frame for internal calculations:
	 * +X: Towards tail
	 * +Y: towards port side (right from aft windows)
	 * +Z: Down
	 * arm_ik_dir and arm_ik_rot define frame oriented along RMS joints; this is slightly rotated from shuttle frame (and from EE frame)
	 */
	VECTOR3 arm_ik_pos, arm_ik_dir, arm_ik_rot;
	VECTOR3 arm_ee_angles; // angles in radians; used for some IK modes
	double joint_angle[6];// angles in degrees
	int joint_motion[6];
	int ee_translation[3];

	// EE, CCTV and light vectors
	VECTOR3 posEE;
	VECTOR3 dirEE;
	VECTOR3 rotEE;
	VECTOR3 rotEEik;
	VECTOR3 posCCTVEE;
	VECTOR3 posCCTVElbow;
	VECTOR3 dirCCTVElbow;
	VECTOR3 rotCCTVElbow;
	VECTOR3 posLight;

	DiscInPort JointSelect[6], DirectDrivePlus, DirectDriveMinus;
	DiscInPort RHCInput[3], THCInput[3];
	DiscInPort RMSMode[12];
	DiscInPort RMSCoarseRateCMD;
	DiscInPort RateHoldPB;// RATE HOLD pushbutton on the RHC grip

	// RATE HOLD state (manual augmented modes only)
	bool bRateHold;
	bool bRateHoldArmed;// hand controllers have been in detent since RATE HOLD was engaged
	bool bRateHoldPBPrev;
	double heldRHC[3], heldTHC[3];// held hand controller commands (volts)

	// mode change detection and SINGLE mode soft stop logic
	int lastRMSMode;
	bool bSingleSoftStopIdle;// SINGLE mode drove a joint into a soft stop: idle until the mode is selected again
	bool singleSoftStopDisabled[6];// joint was inside a soft stop when SINGLE was selected
	bool bAutoModeLogged;

	// panel A8U controls
	DiscInPort BrakesOn;// BRAKES switch in ON
	DiscInPort SafingOn;// SAFING switch in SAFE
	DiscInPort SafingCancel;// SAFING switch in CANCEL (momentary)
	DiscOutPort BrakesApplied;// BRAKES talkback: the brakes hold the joints (switch ON, or no power)
	bool safingLatched;// safing is maintained until it is cancelled
	rmsreact::AccelEstimator reactEst;// acceleration of the payload relative to the orbiter (option RMSReaction)
	rmsreact::AccelEstimator reactArmEst;// acceleration of the center of mass of the arm relative to the orbiter (option RMSReaction)
	VESSEL* reactPayload;
	VECTOR3 posArmCoM;// center of mass of the arm (frame of the mesh)
	DiscInPort AutoSeqProceed;// AUTO SEQ switch in PROCEED
	DiscInPort EnterPB;// ENTER pushbutton
	DiscOutPort ReadyLight, InProgLight;
	DiscOutPort SafingActive;// SAFING talkback: safing in progress (RMS_PANEL line 9)
	int acceptedMode;// mode accepted with ENTER (-2 = not defined yet)
	int effMode;// mode in effect (-1 = idle)
	bool bEnterPrev;
	bool bAutoProceedPrev;

	PDRSControl pdrs;
	PDRSOverride ovr;

	// RMS POWER switch and BACKUP CONTROL (panel A8U)
	DiscInPort PowerPrimary, PowerBackup, PowerOff;// the switch only drives its lines after it has been loaded or operated; if none is set, PRIMARY is assumed
	DiscInPort BackupJoint[6], BackupDirectPlus, BackupDirectMinus, BackupPayloadRelease;
	bool backupDrive;

	// rates of the POR
	bool ratesvalid;
	VECTOR3 lastPos;
	MATRIX3 lastAtt;
	VECTOR3 transRateAct, rotRateAct;// actual rates, Orbiter body axes [ft/s], [deg/s]
	VECTOR3 transRateCmd, rotRateCmd;// commanded rates [ft/s], [deg/s]
	std::vector<AutoPoint> autoseq[20];
	// MCIU and GPC
	pdrs::Logic* gpcLogic;
	pdrs::Mailbox mailbox;
	pdrs::Output gpcOut;
	bool gpcValid;// the software answers
	unsigned short lastCounterOut, lastCounterRead;
	double lastAnswerTime, simtNow;
	bool enterLatch, rateHoldLatch;
	int hwMode;// position of the MODE switch
	void GpcReceive( double simt );
	void GpcSend( bool backup, bool armfree );
	void GpcApply( double simdt, bool backup, bool armfree );

	int autoState;
	int autoPt;// last point reached (1 = first point of the sequence); the OPR CMD mode has only a start and an end point

	double shoulder_brace;
	DiscInPort ShoulderBrace;
	DiscOutPort ShoulderBraceReleased;

	DiscOutPort CWLights[12];
	DiscOutPort ModeLights[12];

	DiscOutPort SoftStopTB;// input inverted to display barberpole when "on"
	DiscOutPort RateMinTB;

	AnimState Grapple_State, Rigid_State, Extend_State;
	DiscInPort EEAuto, EEMan, EERigid, EEDerigid, EEGrapple, EERelease;
	DiscOutPort EECapture, EEExtended, EEClosed, EEOpened, EERigidized, EEDerigidized;
	bool bAutoGrapple, bAutoRelease;

	DiscInPort MasterAlarmPBI;

	bool MasterAlarmOn;
	bool ReachLimit;

	bool arm_moved;
	bool update_vectors, update_angles;

	bool bFirstStep;

	/**
	 * Indicates if RMS is stowed and latched (i.e., not movable).
	 */
	bool stowed_and_latched;

	/**
	 * True if any joint is past its software stop.
	 */
	bool bSoftStop;

	bool bEECapture;
	bool bEEExtended;
	bool bEEClosed;
	bool bEEOpened;
	bool bEERigidized;
	bool bEEDerigidized;

	// for LED displays on panel A8
	DiscOutPort JointAngles[6], EEPosition[3], EEAttitude[3];

	mission::MissionRMS missionrms;
};

#endif //__RMS_H
