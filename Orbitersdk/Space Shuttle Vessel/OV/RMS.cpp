/******* SSV File Modification Notice *******
Date         Developer
2020/04/07   GLS
2020/05/06   GLS
2020/05/07   GLS
2020/05/13   GLS
2020/05/14   GLS
2020/05/15   GLS
2020/05/17   GLS
2020/05/18   GLS
2020/05/21   GLS
2020/05/23   GLS
2020/06/01   GLS
2020/06/20   GLS
2020/06/23   GLS
2020/06/28   GLS
2020/08/24   GLS
2020/09/02   GLS
2020/10/10   GLS
2021/06/16   GLS
2021/06/28   GLS
2021/07/02   GLS
2021/08/23   GLS
2021/08/24   GLS
2021/09/12   GLS
2021/09/20   GLS
2021/10/23   GLS
2021/12/26   GLS
2021/12/29   GLS
2021/12/30   GLS
2022/01/01   GLS
2022/01/02   GLS
2022/01/03   GLS
2022/01/04   GLS
2022/01/10   GLS
2022/01/12   GLS
2022/01/15   GLS
2022/02/19   GLS
2022/03/24   GLS
2022/03/26   GLS
2022/04/09   GLS
2022/05/07   GLS
2022/05/16   GLS
2022/08/05   GLS
2022/09/29   GLS
2022/10/30   GLS
2022/11/01   GLS
2022/11/02   GLS
2022/11/09   GLS
2022/11/12   GLS
2022/11/13   GLS
2023/01/15   GLS
2023/02/12   GLS
2023/02/13   GLS
2023/02/16   GLS
2023/03/26   GLS
2024/12/30   GLS
2026/02/22   GLS
********************************************/
#include "RMS.h"
#include "RMSMath.h"
#include <fstream>
#include <sstream>
#include <string>
#include "ParameterValues.h"
#include "meshres_RMS_Port.h"
#include "../CommonDefs.h"
#include <MathSSV.h>
#include <UtilsSSV.h>
#include "../SSVSound.h"
#include "Atlantis.h"
#include "SSVOptions.h"
#include "ExternalLight.h"
#include <CCTVCamera.h>
#include <CCTVCameraPTU.h>
#include "CCTVCamera_LED.h"
#include "CCTVCameraPTU_LED.h"
#include "RemoteVideoSwitcher.h"
#include "VideoControlUnit.h"
#include <EngConst.h>


const static char* MESHNAME_RMS = "SSV\\RMS_Port";
const static char* MESHNAME_UPPER_PEDESTAL_PORT = "SSV\\MPMUpperPedestal_Port";
const static char* MESHNAME_UPPER_PEDESTAL_STBD = "SSV\\MPMUpperPedestal_Starboard";

constexpr double RMS_ROLLOUT_ANGLE = 19.48;// angle between RMS and vertical when RMS is deployed (in degrees)

const VECTOR3 RMS_MESH_OFFSET = _V( 0.0, 0.0, 0.0 );

// RMS joint positions (stowed, MPM deployed)
const VECTOR3 RMS_SY_JOINT = _V( -2.641549, 0.422581, 6.9797 );// Xo+679.5, Yo-103.998, Zo+433.452
const VECTOR3 RMS_SP_JOINT = _V( -2.7432, 0.709931, 6.9797 );// Xo+679.5, Yo-108.0, Zo+444.765
const VECTOR3 RMS_EP_JOINT = _V( -2.692375, 0.566243, 0.60303 );// Xo+930.55, Yo-105.999, Zo+439.108
const VECTOR3 RMS_WP_JOINT = _V( -2.7432, 0.709931, -6.4569 );// Xo+1208.5, Yo-108.0, Zo+444.765
const VECTOR3 RMS_WY_JOINT = _V( -2.7432, 0.709931, -6.9141 );// Xo+1226.5, Yo-108.0, Zo+444.765
const VECTOR3 RMS_EE_POS = _V( -2.7432, 0.709931, -8.3365 );// Xo+1282.5, Yo-108.0, Zo+444.765

const VECTOR3 RMS_ARM_COM_STOWED = massprops::ArmCoM( RMS_SP_JOINT, RMS_EP_JOINT, RMS_WP_JOINT, RMS_EE_POS );// center of mass of the arm in its place in the default center of mass of the orbiter

const VECTOR3 RMS_SY_AXIS =_V( -0.333478, 0.942758, 0.0 );
const VECTOR3 RMS_SP_AXIS = _V( 0.942758, 0.333478, 0.0 );
const VECTOR3 RMS_EP_AXIS = RMS_SP_AXIS;
const VECTOR3 RMS_WP_AXIS = RMS_SP_AXIS;
const VECTOR3 RMS_WY_AXIS = RMS_SY_AXIS;
const VECTOR3 RMS_WR_AXIS = _V( 0.0, 0.0, 1.0 );

const VECTOR3 RMS_EE_ROT = _V( 0.0, 1.0, 0.0 );

const double RMS_SP_EP_DIST = length(RMS_EP_JOINT-RMS_SP_JOINT);// distance (metres) from SP joint to EP joint
const double RMS_EP_WP_DIST = length(RMS_WP_JOINT-RMS_EP_JOINT);// distance (metres) from EP joint to WP joint
const double RMS_WP_WY_DIST = length(RMS_WY_JOINT-RMS_WP_JOINT);// distance (metres) from WP joint to WY joint
const double RMS_WY_EE_DIST = length(RMS_WY_JOINT-RMS_EE_POS);// distance (metres) from WY joint to EE
const double RMS_SP_NULL_ANGLE = DEG * acos((RMS_SP_JOINT.z-RMS_EP_JOINT.z)/RMS_SP_EP_DIST);// effective angle (degrees) of shoulder boom in null position
const double RMS_EP_NULL_ANGLE = DEG * acos((RMS_EP_JOINT.z-RMS_WP_JOINT.z)/RMS_EP_WP_DIST);// effective angle (degrees) of elbow boom in null position

constexpr double RMS_JOINT_LIMITS[2][6] = {{-180.0, -2.0, -161.0, -121.4, -121.3, -447.0},
					{+180.0, +145.0, +2.4, +121.4, +121.3, +447.0}};
constexpr double RMS_JOINT_SOFTSTOPS[2][6] = {{-177.4, +0.6, -157.6, -116.4, -116.6, -442.0},
					{+177.4, +142.4, -0.4, +116.4, +116.6, +442.0}};
constexpr double RMS_JOINT_REACHLIMITS[2][6] = {{-175.4, +2.6, -155.6, -114.4, -114.6, -440.0},
					{+175.4, +140.4, -2.4, +114.4, +114.6, +440.0}};

//const double RMS_JOINT_ROTATION_SPEED = 1.5;
constexpr double RMS_JOINT_MAX_ROTATION_SPEED[6] = {40.0, 40.0, 40.0, 40.0, 40.0, 40.0}; // mechanical limits; numbers are made up
constexpr double RMS_JOINT_COARSE_ROTATION_SPEEDS[6] = {1.27, 1.27, 1.78, 2.64, 2.64, 2.64}; // software limits; from PL ID 0
constexpr double RMS_JOINT_VERN_ROTATION_SPEEDS[6] = {0.42, 0.42, 0.59, 0.88, 0.88, 0.88}; // software limits; from PL ID 0

// max unloaded rotation/translation speeds from STS-125 PDRS Flight Supplement (PL ID 0)
// RMS arm joint rotation speed (deg/sec)
const double RMS_EE_COARSE_ROTATION_SPEED = 4.76*RAD;
const double RMS_EE_VERN_ROTATION_SPEED = 4.76*RAD;
// RMS IK rotation speed (rad/sec)
constexpr double RMS_EE_COARSE_TRANSLATION_SPEED = 2.0 * FPS2MS;
constexpr double RMS_EE_VERN_TRANSLATION_SPEED = 0.61 * FPS2MS;
// RMS IK translation speed (m/s)

constexpr double RMS_GRAPPLE_SPEED = 0.4;// Time to fully grapple an object (1/s)
const double RMS_RIGID_SPEED = 0.25;// Time to regidize/derigidize arm (1/s)
const double RMS_EXTEND_SPEED = 0.142857;// Time to extend/retract EE (1/s)

constexpr double SHOULDER_BRACE_SPEED = 0.11765;// shoulder brace speed (8.5 seconds)

const VECTOR3 RMS_ELBOW_PTU_POS = _V( -2.554986, 1.311276, 0.032038 );// Xo+953.03, Yo-100.59, Zo+468.44
const VECTOR3 RMS_ELBOW_PTU_PAN_AXIS = _V( 0.422618, 0.906308, 0.0 );// 25º tilt inboard
const VECTOR3 RMS_ELBOW_PTU_TILT_AXIS = _V( 0.906308, -0.422618, 0.0 );// 25º tilt inboard
const VECTOR3 RMS_ELBOW_CAM_DIR = _V( 0.0, 0.0, -1.0 );
const VECTOR3 RMS_ELBOW_CAM_ROT = _V( 0.422618, 0.906308, 0.0 );// 25º tilt inboard

const VECTOR3 RMS_EE_CAM_POS = _V( -2.7432, 0.995681, -7.60498 );// Xo+1253.7, Yo-108.0, Zo+456.015
const VECTOR3 RMS_EE_LIGHT_POS = _V( -2.7432, 1.141731, -7.7142 - 0.068725 );// Xo+1258.0 (center of box, plus offset for box surface), Yo-108.0, Zo+461.765

const VECTOR3 RMS_Z_AXIS = _V( 0.0, 1.0, 0.0 ); // axis along which RMS EE camera & light are mounted
const double RMS_Z_AXIS_ANGLE = RMS_ROLLOUT_ANGLE * RAD; // angle between RMS Z axis and Z axis of IK frame

constexpr double MAX_GRAPPLING_DIST = 0.1016;// [m] max distance between RMS tip and grappling point for successful grappling (4'', not considering longitudinal position)
const double MAX_GRAPPLING_ANGLE = 10.0 * RAD;// [rad] max angle between EE and grapple for successful grappling (limited by max roll error)
constexpr double MRL_MAX_ANGLE_ERROR = 0.1;// [deg] max angular misalignment between MPM and RMS to allow latching


constexpr double LIGHT_RANGE = 20.0;// [m]
const double LIGHT_UMBRA_ANGLE = 40.0 * RAD;// [rad]
const double LIGHT_PENUMBRA_ANGLE = LIGHT_UMBRA_ANGLE + (20.0 * RAD);// [rad]

constexpr double LIGHT_ATT0 = 0.5;// [1]
constexpr double LIGHT_ATT1 = 0.0;// [1]
constexpr double LIGHT_ATT2 = 0.05;// [1]


RMS::RMS( AtlantisSubsystemDirector *_director, const std::string& _ident, bool portside, const mission::MissionRMS& missionrms )
	: MPM( _director, _ident, "GF", portside, MAX_GRAPPLING_DIST, MAX_GRAPPLING_ANGLE ), bFirstStep(true), stowed_and_latched(true), missionrms(missionrms)
{
	joint_angle[SHOULDER_YAW] = 0.0;
	joint_angle[SHOULDER_PITCH] = 0.0;
	joint_angle[ELBOW_PITCH] = 0.0;
	joint_angle[WRIST_PITCH] = 0.0;
	joint_angle[WRIST_YAW] = 0.0;
	joint_angle[WRIST_ROLL] = 0.0;

	posEE = RMS_EE_POS;
	posArmCoM = RMS_ARM_COM_STOWED;
	dirEE = -RMS_WR_AXIS;
	rotEE = RMS_EE_ROT;
	rotEEik = RMS_SY_AXIS;

	posCCTVEE = RMS_EE_CAM_POS;
	posCCTVElbow = RMS_ELBOW_PTU_POS;
	dirCCTVElbow = RMS_ELBOW_CAM_DIR;
	rotCCTVElbow = RMS_ELBOW_CAM_ROT;

	posLight = RMS_EE_LIGHT_POS + RMS_MESH_OFFSET;

	arm_ik_pos = _V(RMS_SP_JOINT.z - RMS_EE_POS.z, 0.0, 0.0);
	arm_ik_dir = _V(1.0, 0.0, 0.0);
	arm_ik_rot = _V(0.0, 0.0, 1.0);
	arm_ee_angles = _V(0.0, 0.0, 0.0);

	// default EE to grapple open and derigidized
	Grapple_State.Set(AnimState::OPEN, 1);
	Rigid_State.Set(AnimState::OPEN, 1);
	Extend_State.Set(AnimState::OPEN, 1);

	bAutoGrapple=false;
	bAutoRelease=false;

	shoulder_brace=0.0; // released

	arm_moved=false;
	update_vectors=false;
	update_angles=false;
	for(int i=0;i<6;i++) joint_motion[i]=0;
	for(int i=0;i<3;i++) ee_translation[i]=0;

	bSoftStop = false;
	bRateHold = false;
	bRateHoldArmed = false;
	bRateHoldPBPrev = false;
	for (int i = 0; i < 3; i++) heldRHC[i] = heldTHC[i] = 0.0;
	lastRMSMode = -1;
	bSingleSoftStopIdle = false;
	for (int i = 0; i < 6; i++) singleSoftStopDisabled[i] = false;
	bAutoModeLogged = false;
	acceptedMode = -2;
	effMode = -1;
	bEnterPrev = false;
	bAutoProceedPrev = false;
	autoState = AUTO_IDLE;
	autoPt = 0;
	pdrs.plid = 0;
	pdrs.eeid = 1;
	pdrs.io = true;// the real software starts with I/O OFF, ON is kept so existing scenarios keep working
	pdrs.softstop_ena = true;
	pdrs.autobrake_ena = true;
	pdrs.posenc_ena = true;
	pdrs.pohs_ena = true;
	for (int i = 0; i < 4; i++) pdrs.autoseq[i] = i + 1;
	pdrs.startpt = 0;
	pdrs.lastpt = 0;
	for (int i = 0; i < 3; i++)
	{
		pdrs.endpos_valid[i] = false;
		pdrs.endpos[i] = 0.0;
		pdrs.endatt_valid[i] = false;
		pdrs.endatt[i] = 0.0;
	}
	pdrs.plinitid = 0;
	pdrs.cmdck = 0;
	pdrs.wrrange = 4;
	pdrs.orbldref = 0;
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
	safingLatched = false;
	reactPayload = NULL;
	backupDrive = false;
	ratesvalid = false;
	lastPos = _V( 0.0, 0.0, 0.0 );
	lastAtt = _M( 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 );
	transRateAct = rotRateAct = transRateCmd = rotRateCmd = _V( 0.0, 0.0, 0.0 );

	bEECapture = false;
	bEEExtended = false;
	bEEOpened = false;
	bEEClosed = false;
	bEERigidized = false;
	bEEDerigidized = false;

	MasterAlarmOn = false;
	ReachLimit = false;

	mesh_index_RMS = MESH_UNDEFINED;
	mesh_index_pedestal_forward = MESH_UNDEFINED;
	mesh_index_pedestal_mid = MESH_UNDEFINED;
	mesh_index_pedestal_aft = MESH_UNDEFINED;

	hMesh_RMS = oapiLoadMeshGlobal( MESHNAME_RMS );
	hMesh_Pedestal = oapiLoadMeshGlobal( MESHNAME_UPPER_PEDESTAL_PORT );

	light = new ExternalLight( STS(), posLight, dirEE, 0.0f, 0.0f, LIGHT_RANGE, LIGHT_ATT0, LIGHT_ATT1, LIGHT_ATT2, LIGHT_UMBRA_ANGLE, LIGHT_PENUMBRA_ANGLE, INCANDESCENT );

	CreateCCTV();
}

RMS::~RMS()
{
	delete cameraElbow;
	delete cameraWrist;
	delete videoswitcher;
}

void RMS::Realize()
{
	MPM::Realize();

	DiscreteBundle* pBundle=BundleManager()->CreateBundle("RMS_EE", 16);
	EEGrapple.Connect(pBundle, 0);
	EERelease.Connect(pBundle, 1);
	EEAuto.Connect(pBundle, 2);
	EEMan.Connect(pBundle, 3);
	EERigid.Connect(pBundle, 4);
	EEDerigid.Connect(pBundle, 5);
	EECapture.Connect(pBundle, 6);
	EEExtended.Connect(pBundle, 7);
	EEClosed.Connect(pBundle, 8);
	EEOpened.Connect(pBundle, 9);
	EERigidized.Connect(pBundle, 10);
	EEDerigidized.Connect(pBundle, 11);
	RateHoldPB.Connect(pBundle, 12);

	pBundle = BundleManager()->CreateBundle( "RMS_PANEL", 16 );
	BrakesOn.Connect( pBundle, 0 );
	SafingOn.Connect( pBundle, 1 );
	AutoSeqProceed.Connect( pBundle, 2 );
	EnterPB.Connect( pBundle, 3 );
	PowerPrimary.Connect( pBundle, 4 );
	PowerBackup.Connect( pBundle, 5 );
	PowerOff.Connect( pBundle, 6 );
	SafingActive.Connect( pBundle, 9 );
	SafingCancel.Connect( pBundle, 10 );
	BrakesApplied.Connect( pBundle, 11 );

	pBundle = BundleManager()->CreateBundle( "RMS_BACKUP", 16 );
	for (int i = 0; i < 6; i++) BackupJoint[i].Connect( pBundle, i );
	BackupDirectPlus.Connect( pBundle, 6 );
	BackupDirectMinus.Connect( pBundle, 7 );
	BackupPayloadRelease.Connect( pBundle, 8 );

	LoadAutoSequences();

	pBundle=BundleManager()->CreateBundle("RMS", 16);
	ShoulderBrace.Connect(pBundle, 4);
	ShoulderBraceReleased.Connect(pBundle, 5);
	RMSSelect.Connect(pBundle, 6);
	MasterAlarmPBI.Connect( pBundle, 7 );

	pBundle=BundleManager()->CreateBundle("RMS_DATA", 16);
	for(int i=0;i<6;i++) JointAngles[i].Connect(pBundle, i);
	for(int i=0;i<3;i++) {
		EEPosition[i].Connect(pBundle, i+6);
		EEAttitude[i].Connect(pBundle, i+9);
	}

	pBundle=BundleManager()->CreateBundle("RMS_HC_INPUT", 16);
	for(int i=0;i<3;i++) {
		RHCInput[i].Connect(pBundle, i);
		THCInput[i].Connect(pBundle, i+3);
	}

	pBundle=BundleManager()->CreateBundle("RMS_SINGLE_JOINT", 16);
	for(int i=0;i<6;i++) {
		JointSelect[i].Connect(pBundle, i);
	}
	// indexes 6 and 7 and used for temps
	DirectDrivePlus.Connect(pBundle, 8);
	DirectDriveMinus.Connect(pBundle, 9);

	pBundle = BundleManager()->CreateBundle( "RMS_MODE", 16 );
	for (int i = 0; i < 12; i++) RMSMode[i].Connect( pBundle, i );
	RMSCoarseRateCMD.Connect( pBundle, 12 );

	pBundle = BundleManager()->CreateBundle( "RMS_CWLIGHTS_TB", 16 );
	for (int i = 0; i < 12; i++) CWLights[i].Connect( pBundle, i );
	SoftStopTB.Connect( pBundle, 12 );
	RateMinTB.Connect( pBundle, 13 );

	pBundle = BundleManager()->CreateBundle( "RMS_MODELIGHTS", 16 );
	for (int i = 0; i < 12; i++) ModeLights[i].Connect( pBundle, i );
	ReadyLight.Connect( pBundle, 12 );
	InProgLight.Connect( pBundle, 13 );

	AddMesh();

	{
		VideoControlUnit* pVCU = static_cast<VideoControlUnit*>(director->GetSubsystemByName( "VideoControlUnit" ));
		pVCU->AddCamera( videoswitcher, portside ? IN_PORT_RMS : IN_STBD_RMS );

		pBundle = BundleManager()->CreateBundle( "VCU_input_3", 16 );
		videoswitcher->ConnectSelSwitch( pBundle, 13 );// TODO handle stbd switch

		pBundle = BundleManager()->CreateBundle( "VCU_output", 16 );
		videoswitcher->ConnectPowerOnOff( pBundle, portside ? 9 : 8 );

		pBundle = STS()->BundleManager()->CreateBundle( "CAMERA_POWER", 16 );
		cameraElbow->ConnectPowerCameraPTU( pBundle, portside ? 12 : 9 );
		cameraElbow->ConnectPowerHeater( pBundle, portside ? 13 : 10 );
		cameraElbow->ConnectPowerPTUHeater( pBundle, portside ? 14 : 11 );

		CCTVCameraPTU_LED* ledptu = dynamic_cast<CCTVCameraPTU_LED*>(cameraElbow);
		if (ledptu)
		{
			ledptu->DefineMeshGroup( mesh_index_RMS, GRP_ELBOW_CAMERA_LED_ILLUMINATOR_FRONT_RMS_Port );
			ledptu->ConnectLEDPower( pBundle, portside ? 14 : 11 );
		}

		cameraWrist->ConnectPowerCameraPTU( pBundle, portside ? 12 : 9 );
		cameraWrist->ConnectPowerHeater( pBundle, portside ? 13 : 10 );

		CCTVCamera_LED* led = dynamic_cast<CCTVCamera_LED*>(cameraWrist);
		if (led)
		{
			led->DefineMeshGroup( mesh_index_RMS, GRP_WRIST_CAMERA_LED_ILLUMINATOR_FRONT_RMS_Port );
			led->ConnectLEDPower( pBundle, portside ? 13 : 10 );
		}
	}

	DefineAnimations();

	// add end effector light
	pBundle = BundleManager()->CreateBundle( "PLB_LIGHTS", 16 );
	light->DefineMeshGroup( mesh_index_RMS, GRP_LIGHT_RMS_Port );
	light->DefineState( 1, 0.5f, 0.0f, 1.0f, pBundle, 9 );
	return;
}

void RMS::AddMesh( void )
{
	VECTOR3 ofs = STS()->GetOrbiterCoGOffset();
	mesh_index_RMS = STS()->AddMesh( hMesh_RMS, &ofs );
	STS()->SetMeshVisibilityMode( mesh_index_RMS, MESHVIS_EXTERNAL | MESHVIS_VC | MESHVIS_EXTPASS );

	ofs = STS()->GetOrbiterCoGOffset() + PEDESTAL_OFFSET_PORT_FORWARD;
	mesh_index_pedestal_forward = STS()->AddMesh( hMesh_Pedestal, &ofs );
	STS()->SetMeshVisibilityMode( mesh_index_pedestal_forward, MESHVIS_EXTERNAL | MESHVIS_VC | MESHVIS_EXTPASS );

	ofs = STS()->GetOrbiterCoGOffset() + PEDESTAL_OFFSET_PORT_MID;
	mesh_index_pedestal_mid = STS()->AddMesh( hMesh_Pedestal, &ofs );
	STS()->SetMeshVisibilityMode( mesh_index_pedestal_mid, MESHVIS_EXTERNAL | MESHVIS_VC | MESHVIS_EXTPASS );

	ofs = STS()->GetOrbiterCoGOffset() + PEDESTAL_OFFSET_PORT_AFT;
	mesh_index_pedestal_aft = STS()->AddMesh( hMesh_Pedestal, &ofs );
	STS()->SetMeshVisibilityMode( mesh_index_pedestal_aft, MESHVIS_EXTERNAL | MESHVIS_VC | MESHVIS_EXTPASS );
	return;
}

void RMS::DefineAnimations( void )
{
	const VECTOR3 PitchAxis = RotateVectorZ( _V( 1.0, 0.0, 0.0 ), RMS_ROLLOUT_ANGLE );
	const VECTOR3 YawAxis = RotateVectorZ( _V( 0.0, 1.0, 0.0 ), RMS_ROLLOUT_ANGLE );

	// upper pedestals
	static UINT BaseGrp = GRP_BASE_RMS_Port;
	MGROUP_TRANSFORM* pBaseAnim = new MGROUP_TRANSFORM( mesh_index_RMS, &BaseGrp, 1 );
	STS()->AddAnimationComponent( anim_mpm, 0.0, 1.0, pBaseAnim, mpmparent );
	SaveAnimation( pBaseAnim );

	MGROUP_TRANSFORM* pUpperPedestalFWDAnim = new MGROUP_TRANSFORM( mesh_index_pedestal_forward, NULL, 0 );
	STS()->AddAnimationComponent( anim_mpm, 0.0, 1.0, pUpperPedestalFWDAnim, mpmparent );
	SaveAnimation( pUpperPedestalFWDAnim );
	MGROUP_TRANSFORM* pUpperPedestalMIDAnim = new MGROUP_TRANSFORM( mesh_index_pedestal_mid, NULL, 0 );
	STS()->AddAnimationComponent( anim_mpm, 0.0, 1.0, pUpperPedestalMIDAnim, mpmparent );
	SaveAnimation( pUpperPedestalMIDAnim );
	MGROUP_TRANSFORM* pUpperPedestalAFTAnim = new MGROUP_TRANSFORM( mesh_index_pedestal_aft, NULL, 0 );
	STS()->AddAnimationComponent( anim_mpm, 0.0, 1.0, pUpperPedestalAFTAnim, mpmparent );
	SaveAnimation( pUpperPedestalAFTAnim );

	// shoulder yaw
	static UINT RMSShoulderYawGrp[2] = {GRP_SHOULDER_YAW_RMS_Port, GRP_SHOULDER_CABLE_HARNESS_RMS_Port};
	MGROUP_ROTATE* pRMS_sy_anim = new MGROUP_ROTATE( mesh_index_RMS, RMSShoulderYawGrp, 2, RMS_SY_JOINT, YawAxis, static_cast<float>(-360.0 * RAD) );// [-180.0,+180.0]
	anim_joint[SHOULDER_YAW] = STS()->CreateAnimation( 0.5 );
	ANIMATIONCOMPONENT_HANDLE parent = STS()->AddAnimationComponent( anim_joint[SHOULDER_YAW], 0.0, 1.0, pRMS_sy_anim, mpmparent );
	SaveAnimation( pRMS_sy_anim );

	// shoulder pitch
	static UINT RMSShoulderPitchGrp[2] = {GRP_UPPER_ARM_BOOM_RMS_Port, GRP_MPM_RETENTION_MECHANISM_UPPER_ARM_BOOM_RMS_Port};
	MGROUP_ROTATE* pRMS_sp_anim = new MGROUP_ROTATE( mesh_index_RMS, RMSShoulderPitchGrp, 2, RMS_SP_JOINT, PitchAxis, static_cast<float>(147.0 * RAD) );// [-2.0,+145.0]
	anim_joint[SHOULDER_PITCH] = STS()->CreateAnimation( 2.0 / 147.0 );
	parent = STS()->AddAnimationComponent( anim_joint[SHOULDER_PITCH], 0.0, 1.0, pRMS_sp_anim, parent );
	SaveAnimation( pRMS_sp_anim );

	// elbow pitch
	static UINT RMSElbowPitchGrp[4] = {GRP_LOWER_ARM_BOOM_RMS_Port, GRP_MPM_RETENTION_MECHANISM_LOWER_ARM_BOOM_RMS_Port, GRP_ELBOW_CAMERA_PTU_BASE_GOLD_RMS_Port, GRP_ELBOW_CAMERA_PTU_BASE_SILVER_RMS_Port};
	MGROUP_ROTATE* pRMS_ep_anim = new MGROUP_ROTATE( mesh_index_RMS, RMSElbowPitchGrp, 4, RMS_EP_JOINT, PitchAxis, static_cast<float>(163.4 * RAD) );// [-161.0,+2.4]
	anim_joint[ELBOW_PITCH] = STS()->CreateAnimation( 161.0 / 163.4 );
	parent = STS()->AddAnimationComponent( anim_joint[ELBOW_PITCH], 0.0, 1.0, pRMS_ep_anim, parent );
	SaveAnimation( pRMS_ep_anim );

	// RMS elbow camera
	static UINT RMSElbowCamPanGrp[4] = {GRP_ELBOW_CAMERA_PTU_GOLD_RMS_Port, GRP_ELBOW_CAMERA_PTU_SILVER_RMS_Port, GRP_ELBOW_CAMERA_CABLE_RMS_Port, GRP_ELBOW_CAMERA_CONNECTOR_RMS_Port};
	MGROUP_ROTATE* pRMSElbowCamPan = new MGROUP_ROTATE( mesh_index_RMS, RMSElbowCamPanGrp, 4, RMS_ELBOW_PTU_POS, RMS_ELBOW_PTU_PAN_AXIS, static_cast<float>((PLB_CAM_PAN_MAX - PLB_CAM_PAN_MIN) * RAD) );
	UINT anim_CamElbowPan = STS()->CreateAnimation( 0.5 );
	ANIMATIONCOMPONENT_HANDLE parent2 = STS()->AddAnimationComponent( anim_CamElbowPan, 0.0, 1.0, pRMSElbowCamPan, parent );
	SaveAnimation( pRMSElbowCamPan );

	static UINT RMSElbowCamTiltGrp[7] = {GRP_ELBOW_CAMERA_MINUS506MINUS508_RMS_Port, GRP_ELBOW_CAMERA_CTVCITVC_RMS_Port, GRP_ELBOW_CAMERA_LED_CABLE_1_RMS_Port, GRP_ELBOW_CAMERA_LED_CABLE_2_RMS_Port, GRP_ELBOW_CAMERA_LED_ILLUMINATOR_BODY_RMS_Port, GRP_ELBOW_CAMERA_LED_ILLUMINATOR_FRONT_RMS_Port, GRP_ELBOW_CAMERA_LED_POWER_BOX_RMS_Port};
	MGROUP_ROTATE* pRMSElbowCamTilt = new MGROUP_ROTATE( mesh_index_RMS, RMSElbowCamTiltGrp, 7, RMS_ELBOW_PTU_POS, RMS_ELBOW_PTU_TILT_AXIS, static_cast<float>((PLB_CAM_TILT_MAX - PLB_CAM_TILT_MIN) * RAD) );
	UINT anim_CamElbowTilt = STS()->CreateAnimation( PLB_CAM_TILT_MIN / (PLB_CAM_TILT_MIN - PLB_CAM_TILT_MAX) );
	STS()->AddAnimationComponent( anim_CamElbowTilt, 0.0, 1.0, pRMSElbowCamTilt, parent2 );
	SaveAnimation( pRMSElbowCamTilt );

	cameraElbow->DefineAnimations( 0.0, -25.0, anim_CamElbowPan, anim_CamElbowTilt );

	// wrist pitch
	static UINT RMSWristPitchGrp[1] = {GRP_WRIST_PITCH_RMS_Port};
	MGROUP_ROTATE* pRMS_wp_anim = new MGROUP_ROTATE( mesh_index_RMS, RMSWristPitchGrp, 1, RMS_WP_JOINT, PitchAxis, static_cast<float>(242.8 * RAD) );// [-121.4,+121.4]
	anim_joint[WRIST_PITCH] = STS()->CreateAnimation( 0.5 );
	parent = STS()->AddAnimationComponent( anim_joint[WRIST_PITCH], 0.0, 1.0, pRMS_wp_anim, parent );
	SaveAnimation( pRMS_wp_anim );

	// wrist yaw
	static UINT RMSWristYawGrp[1] = {GRP_WRIST_YAW_RMS_Port};
	MGROUP_ROTATE* pRMS_wy_anim = new MGROUP_ROTATE( mesh_index_RMS, RMSWristYawGrp, 1, RMS_WY_JOINT, YawAxis, static_cast<float>(242.6 * RAD) );// [-121.3,+121.3]
	anim_joint[WRIST_YAW] = STS()->CreateAnimation( 0.5 );
	parent = STS()->AddAnimationComponent( anim_joint[WRIST_YAW], 0.0, 1.0, pRMS_wy_anim, parent );
	SaveAnimation( pRMS_wy_anim );

	// wrist roll
	static UINT RMSEndEffectorGrp[15] = {GRP_WRIST_ROLL_RMS_Port, GRP_CAMERA_LIGHT_BRACKET_RMS_Port, GRP_WRIST_CAMERA_MINUS506MINUS508_RMS_Port, GRP_WRIST_CAMERA_CTVCITVC_RMS_Port, GRP_WRIST_CAMERA_CABLE_RMS_Port, GRP_WRIST_CAMERA_CONNECTORS_RMS_Port, GRP_WRIST_CAMERA_LED_CABLE_1_RMS_Port, GRP_WRIST_CAMERA_LED_CABLE_2_RMS_Port, GRP_WRIST_CAMERA_LED_ILLUMINATOR_BODY_RMS_Port, GRP_WRIST_CAMERA_LED_ILLUMINATOR_FRONT_RMS_Port, GRP_WRIST_CAMERA_LED_POWER_BOX_RMS_Port, GRP_LIGHT_RMS_Port, GRP_LIGHT_HOUSING_RMS_Port, GRP_STANDARD_END_EFFECTOR_RMS_Port, GRP_MPM_RETENTION_MECHANISM_END_EFFECTOR_RMS_Port};
	MGROUP_ROTATE* pRMS_wr_anim = new MGROUP_ROTATE( mesh_index_RMS, RMSEndEffectorGrp, 15, RMS_EE_POS, _V( 0.0, 0.0, 1.0 ), static_cast<float>(894.0 * RAD) );// [-447.0,+447.0]
	anim_joint[WRIST_ROLL] = STS()->CreateAnimation( 0.5 );
	STS()->AddAnimationComponent( anim_joint[WRIST_ROLL], 0.0, 1.0, pRMS_wr_anim, parent );
	SaveAnimation( pRMS_wr_anim );

	//if joint angles were loaded from scenario file,
	if(arm_moved)
	{
		SetJointAngle( SHOULDER_YAW, joint_angle[SHOULDER_YAW] );
		SetJointAngle( SHOULDER_PITCH, joint_angle[SHOULDER_PITCH] );
		SetJointAngle( ELBOW_PITCH, joint_angle[ELBOW_PITCH] );
		SetJointAngle( WRIST_PITCH, joint_angle[WRIST_PITCH] );
		SetJointAngle( WRIST_YAW, joint_angle[WRIST_YAW] );
		SetJointAngle( WRIST_ROLL, joint_angle[WRIST_ROLL] );
		CalcVectors();
	}
	return;
}

void RMS::CreateAttachment()
{
	if(!hAttach)
		hAttach=STS()->CreateAttachment( false, STS()->GetOrbiterCoGOffset() + posEE+RMS_MESH_OFFSET, dirEE, RMS_EE_ROT, "SSV_GF" );
}

void RMS::UpdateAttachment( void )
{
	if (hAttach) STS()->SetAttachmentParams( hAttach, STS()->GetOrbiterCoGOffset() + posEE + RMS_MESH_OFFSET, dirEE, rotEE );

	// also update light and camera position
	UpdateEELight();
	return;
}

void RMS::ShiftCG( const VECTOR3& shift )
{
	UpdateEELight();
	return;
}

bool RMS::Movable( void ) const
{
	return (Powered() && Eq( shoulder_brace, 0.0, 0.01 ) && !stowed_and_latched && !STS()->GravityLoaded())
		&& (hPayloadAttachment == NULL || (!doubleAttached && PayloadIsFree()));
}

void RMS::OnPreStep(double simt, double simdt, double mjd)
{
	MPM::OnPreStep(simt, simdt, mjd);

	// update light
	light->TimeStep( simdt );

	videoswitcher->TimeStep();

	if (bFirstStep) CheckRFL();

	// reaction of the arm on the orbiter: the arm accelerates the payload and its own mass (relative to the orbiter), and the orbiter feels the
	// opposite force at the same places (option RMSReaction). The positions are taken in the frame of the mesh, so that a shift of the c.g.
	// (option RMSCenterOfMass) is not taken as an acceleration.
	if (STS()->GetOptions()->RMSReaction() && (simdt > 0.0) && (simdt < 0.5))
	{
		const VECTOR3 ofs = STS()->GetOrbiterCoGOffset();// local frame = frame of the mesh + ofs
		VECTOR3 acc;
		const VECTOR3 armPos = posArmCoM + RMS_MESH_OFFSET;
		if (reactArmEst.Update( armPos, simdt, acc )) STS()->AddForce( rmsreact::ReactionForce( massprops::ARM_MASS, acc ), armPos + ofs );

		if (attachedPayload != NULL)
		{
			if (reactPayload != attachedPayload)
			{
				reactEst.Reset();
				reactPayload = attachedPayload;
			}
			VECTOR3 gpos, lpos;
			attachedPayload->GetGlobalPos( gpos );
			STS()->Global2Local( gpos, lpos );
			if (reactEst.Update( lpos - ofs, simdt, acc )) STS()->AddForce( rmsreact::ReactionForce( attachedPayload->GetMass(), acc ), lpos );
		}
		else
		{
			reactEst.Reset();
			reactPayload = NULL;
		}
	}
	else
	{
		reactEst.Reset();
		reactArmEst.Reset();
		reactPayload = NULL;
	}

	// make sure RMS is powered and can be operated
	if(!Powered()) return;

	// MODE switch position
	int curmode = -1;
	for (int i = 0; i < 12; i++)
	{
		if (RMSMode[i].IsSet()) curmode = i;
	}
	if (ovr.mode_ena && (ovr.mode >= 0)) curmode = ovr.mode;// MODE SW OVRD
	// the mode selected with MODE is entered with ENTER (not needed for DIRECT); if the MODE switch is moved, the software is idle until ENTER is pushed
	bool enter = EnterPB.IsSet() || ovr.enter;
	ovr.enter = false;
	if (acceptedMode == -2) acceptedMode = curmode;// nothing stored in the scenario
	if (enter && !bEnterPrev) acceptedMode = curmode;
	bEnterPrev = enter;
	if (curmode == MODE_DIRECT) effMode = MODE_DIRECT;
	else if (curmode == acceptedMode) effMode = curmode;
	else effMode = -1;

	// with RMS POWER in BACKUP, only the BACKUP CONTROL switches work (no computer supported modes)
	const bool backup = PowerBackup.IsSet() && !PowerPrimary.IsSet();
	if (backup) effMode = -1;

	// detect mode changes: RATE HOLD is cancelled and the soft stop logic of SINGLE mode is initialized
	if (effMode != lastRMSMode)
	{
		lastRMSMode = effMode;
		autoState = AUTO_IDLE;
		autoPt = 0;
		bRateHold = false;
		bSingleSoftStopIdle = false;
		bAutoModeLogged = false;
		// soft stops are disabled for the joints that are already in a soft stop when SINGLE mode is selected
		for (int i = SHOULDER_YAW; i <= WRIST_ROLL; i++)
		{
			singleSoftStopDisabled[i] = (joint_angle[i] < RMS_JOINT_SOFTSTOPS[0][i]) || (joint_angle[i] > RMS_JOINT_SOFTSTOPS[1][i]);
		}
	}

	// the brakes hold the arm; SAFING brings the arm to rest until it is cancelled
	// SAFING: the switch in SAFE brings the arm to rest, and the safing is maintained when the switch goes back to AUTO until it is cancelled
	// with the switch momentarily in CANCEL (or SAFING CAN of the PDRS OVERRIDE display)
	safingLatched = RMSMath::SafingLatch( safingLatched, SafingOn.IsSet(), SafingCancel.IsSet(), ovr.safing_cancel );
	if (!SafingOn.IsSet()) ovr.safing_cancel = false;
	const bool safing = safingLatched;
	if (safing) SafingActive.SetLine();
	else SafingActive.ResetLine();
	// the brakes are released with power on: without power they clamp the joints
	if (RMSMath::BrakesApplied( BrakesOn.IsSet(), Powered() )) BrakesApplied.SetLine();
	else BrakesApplied.ResetLine();
	bool armfree = Movable() && !BrakesOn.IsSet() && !safing;
	if (!armfree) bRateHold = false;

	// RATE HOLD pushbutton (RHC grip): push to hold the commanded rates, push again to release
	bool ratehold_pb = RateHoldPB.IsSet();
	if (ratehold_pb && !bRateHoldPBPrev)
	{
		if (bRateHold) bRateHold = false;
		else if (armfree && ManualAugmentedMode())
		{
			for (int i = 0; i < 3; i++)
			{
				heldRHC[i] = RHCInput[i].GetVoltage();
				heldTHC[i] = THCInput[i].GetVoltage();
			}
			bRateHoldArmed = false;
			bRateHold = true;
		}
	}
	bRateHoldPBPrev = ratehold_pb;

	transRateCmd = rotRateCmd = _V( 0.0, 0.0, 0.0 );

	// BACKUP CONTROL: the selected joint is driven with the DIRECT DRIVE switch
	if (backup && armfree)
	{
		int dir = BackupDirectPlus.IsSet() ? 1 : (BackupDirectMinus.IsSet() ? -1 : 0);
		int joint = -1;
		for (int i = 0; i < 6; i++)
		{
			if (BackupJoint[i].IsSet()) joint = i;
		}
		if ((dir != 0) && (joint != -1))
		{
			backupDrive = true;
			DriveSingleJoint( joint, dir, simdt );
			backupDrive = false;
			update_vectors = true;
		}
	}

	//rotate joints
	if(armfree) {
		if(ModeIs(MODE_SINGLE) || ModeIs(MODE_DIRECT)) { // joint-by-joint drive
			int dir = 0;
			if(DirectDrivePlus) dir = 1;
			else if(DirectDriveMinus) dir = -1;
			if(dir != 0) {
				int joint=GetSelectedJoint();
				if(joint!=-1) DriveSingleJoint(joint, dir, simdt);
				update_vectors=true;
			}
		}
		else if(ModeIs(MODE_TEST)) {
			// TEST is equivalent to ORB LD, but with zero joint rates: the hand controllers can be checked out and the arm is limp
		}
		else if(ManualAugmentedMode()) {
			VECTOR3 newPos = arm_ik_pos;
			VECTOR3 newDir = arm_ik_dir;
			VECTOR3 newRot = arm_ik_rot;
			bool rotateEE=false, translateEE=false;
			const double loadscale = GetLoadScale();

			// hand controller commands (volts, 5V = full deflection); the rates are proportional to the deflection
			double rhc[3], thc[3];
			bool detent = true;
			for(int i=0;i<3;i++) {
				rhc[i] = RHCInput[i].GetVoltage();
				thc[i] = THCInput[i].GetVoltage();
				if(!Eq(rhc[i], 0.0, 0.05) || !Eq(thc[i], 0.0, 0.05)) detent = false;
			}
			if(bRateHold) {
				if(detent) bRateHoldArmed = true;
				for(int i=0;i<3;i++) {
					// after the hand controllers return to detent, further deflection biases the held rates
					double rhc_bias = bRateHoldArmed ? rhc[i] : 0.0;
					double thc_bias = bRateHoldArmed ? thc[i] : 0.0;
					rhc[i] = max(-5.0, min(5.0, heldRHC[i] + rhc_bias));
					thc[i] = max(-5.0, min(5.0, heldTHC[i] + thc_bias));
				}
			}

			// EE rotation
			VECTOR3 change=_V(0.0, 0.0, 0.0);
			for(int i=0;i<3;i++) {
				if(!Eq(rhc[i], 0.0, 0.05)) {
					if (RateCoarse()) change.data[i]+=(rhc[i]/5.0)*simdt*RMS_EE_COARSE_ROTATION_SPEED*loadscale;
					else change.data[i]+=(rhc[i]/5.0)*simdt*RMS_EE_VERN_ROTATION_SPEED*loadscale;
					rotateEE=true;
				}
			}
			if(rotateEE)
			{
				Rotate(change, newPos, newDir, newRot);
				if (!ModeIs(MODE_END_EFF)) rotRateCmd = _V( change.data[ROLL], change.data[PITCH], change.data[YAW] ) * (DEG / simdt);
			}

			// EE translation
			change=_V(0.0, 0.0, 0.0);
			for(int i=0;i<3;i++) {
				if(!Eq(thc[i], 0.0, 0.05)) {
					if (RateCoarse()) change.data[i]+=(thc[i]/5.0)*simdt*RMS_EE_COARSE_TRANSLATION_SPEED*loadscale;
					else change.data[i]+=(thc[i]/5.0)*simdt*RMS_EE_VERN_TRANSLATION_SPEED*loadscale;
					translateEE=true;
				}
				else if(ee_translation[i]!=0) {
					if (RateCoarse()) change.data[i]+=ee_translation[i]*simdt*RMS_EE_COARSE_TRANSLATION_SPEED*loadscale;
					else change.data[i]+=ee_translation[i]*simdt*RMS_EE_VERN_TRANSLATION_SPEED*loadscale;
					ee_translation[i]=0;
					translateEE=true;
				}
			}
			if(translateEE)
			{
				Translate(change, newPos);
				if (!ModeIs(MODE_END_EFF)) transRateCmd = change / (simdt * 0.3048);
			}

			if(translateEE || rotateEE) MoveEE(newPos, newDir, newRot, simdt);
		}
	}

	// OPR CMD, AUTO 1-4
	RunAutoModes( simdt, armfree );

	CheckSoftwareStop();
	// the software drops out of mode (idle) when a joint reaches a soft stop, which cancels RATE HOLD
	if (bSoftStop) bRateHold = false;

	const bool backuprelease = backup && BackupPayloadRelease.IsSet();
	const bool eeAuto = EEAuto.IsSet() || backuprelease;
	const bool eeRelease = EERelease.IsSet() || backuprelease;
	if(eeAuto || EEMan) {
		if(Grapple_State.Moving()) {
			Grapple_State.Move(simdt*RMS_GRAPPLE_SPEED);

			if(Grapple_State.Closed()) {
				if(!STS()->GetAttachmentStatus(hAttach)) AttachPayload();
				bEEClosed = true;
				if(eeAuto) AutoGrappleSequence();
			}
			else if(Grapple_State.Open()) {
				bEEOpened = true;
				if(eeAuto) AutoReleaseSequence();
			}
			else {
				if(IsLatched()) DetachPayload();
				bEEClosed = false;
				bEEOpened = false;
			}
		}
		if(Extend_State.Moving()) {
			Extend_State.Move(simdt*RMS_EXTEND_SPEED);
			if(Extend_State.Open()) {
				bEEExtended = true;
				if(eeAuto) AutoReleaseSequence();
			}
			else if(Extend_State.Closed() && eeAuto) AutoGrappleSequence();
			else bEEExtended = false;
		}
		if(Rigid_State.Moving()) {
			Rigid_State.Move(simdt*RMS_RIGID_SPEED);
			if(Rigid_State.Open()) {
				bEEDerigidized = true;
				if(eeAuto) AutoReleaseSequence();
			}
			else if(Rigid_State.Closed()) {
				bEERigidized = true;
				if(eeAuto) AutoGrappleSequence();
			}
			else {
				bEEDerigidized = false;
				bEERigidized = false;
			}
		}

		if(eeAuto) {
			if(EEGrapple) {
				bAutoGrapple=true;
				bAutoRelease=false;
				AutoGrappleSequence();
			}
			else if(eeRelease) {
				bAutoRelease=true;
				bAutoGrapple=false;
				AutoReleaseSequence();
			}
		}
		else { //EEMan must be set
			if(EEGrapple) Grapple_State.action=AnimState::CLOSING;
			else if(eeRelease) Grapple_State.action=AnimState::OPENING;
			else if(Grapple_State.Moving()) Grapple_State.action=AnimState::STOPPED;

			if(EERigid) {
				if(!Extend_State.Closed()) Extend_State.action=AnimState::CLOSING;
				else if(!Rigid_State.Closed()) Rigid_State.action=AnimState::CLOSING;
			}
			else if(EEDerigid) {
				if(!Rigid_State.Open()) Rigid_State.action=AnimState::OPENING;
				else if(!Extend_State.Open()) Extend_State.action=AnimState::OPENING;
			}
			else {
				if(Extend_State.Moving()) Extend_State.action=AnimState::STOPPED;
				if(Rigid_State.Moving()) Rigid_State.action=AnimState::STOPPED;
			}
		}
	}
	else {
		if(Grapple_State.Moving()) Grapple_State.action=AnimState::STOPPED;
		if(Extend_State.Moving()) Extend_State.action=AnimState::STOPPED;
		if(Rigid_State.Moving()) Rigid_State.action=AnimState::STOPPED;
	}

	if(ShoulderBrace) {
		if(shoulder_brace>0.0) {
			shoulder_brace=max(shoulder_brace-simdt*SHOULDER_BRACE_SPEED, 0.0);
		}
	}

	if (MasterAlarmPBI.IsSet())
	{
		MasterAlarmOn = false;
		SoundStop( STS()->GetSound(), CW_TONE_RMS_SOUND );
	}

	if (bFirstStep)
	{
		// set lines
		if(IsLatched()) bEECapture = true;
		if(Extend_State.Open()) bEEExtended = true;
		if(Grapple_State.Open()) bEEOpened = true;
		else if(Grapple_State.Closed()) bEEClosed = true;
		if(Rigid_State.Closed()) bEERigidized = true;
		else if(Rigid_State.Open()) bEEDerigidized = true;

		SoftStopTB.SetLine( (int)bSoftStop * 5.0f );
		RateMinTB.SetLine( (int)RateCoarse() * 5.0f );

		EECapture.SetLine( (int)bEECapture * 5.0f );
		EEExtended.SetLine( (int)bEEExtended * 5.0f );
		EEClosed.SetLine( (int)bEEClosed * 5.0f );
		EEOpened.SetLine( (int)bEEOpened * 5.0f );
		EERigidized.SetLine( (int)bEERigidized * 5.0f );
		EEDerigidized.SetLine( (int)bEEDerigidized * 5.0f );

		if(Eq(shoulder_brace, 0.0, 0.01)) ShoulderBraceReleased.SetLine();
	}
}

void RMS::OnPostStep(double simt, double simdt, double mjd)
{
	MPM::OnPostStep(simt, simdt, mjd);

	// update end effector light position/direction
	if (arm_moved || mpm_moved)
	{
		CalcVectors();
		UpdateEELight();

		cameraElbow->SetPhysicalParams( posCCTVElbow, dirCCTVElbow, rotCCTVElbow );
		cameraWrist->SetPhysicalParams( posCCTVEE, dirEE, rotEE );

		// if arm was moved, update attachment position and IK vectors/angles
		// due to bug in orbiter_ng/D3D9 client, this needs to be done on second timestep
		if(hAttach) STS()->SetAttachmentParams(hAttach, STS()->GetOrbiterCoGOffset()+posEE+RMS_MESH_OFFSET, dirEE, rotEE);

		CheckRFL();

		/*** Update output lines to LEDs ***/
		// calculate position
		VECTOR3 ee_pos_output = (posEE - RMS_SP_JOINT) / IN2M;
		ee_pos_output = _V(ee_pos_output.z, ee_pos_output.x, -ee_pos_output.y) + _V( -679.5, -108.0, -444.77 );

		// calculate attitude
		VECTOR3 ee_frame_x, ee_frame_y, ee_frame_z; // reference frame define by EE direction
		ee_frame_x=ConvertVectorToRMSFrame( dirEE );
		ee_frame_z=ConvertVectorToRMSFrame( -rotEE );
		ee_frame_y=crossp(ee_frame_z, ee_frame_x);
		MATRIX3 ee_frame_mat = _M(ee_frame_x.x, ee_frame_y.x, ee_frame_z.x,
								  ee_frame_x.y, ee_frame_y.y, ee_frame_z.y,
								  ee_frame_x.z, ee_frame_y.z, ee_frame_z.z);
		VECTOR3 ee_att_output = GetYZX_PYRAnglesFromMatrix(ee_frame_mat);
		for(int i=0;i<3;i++) {
			if(ee_att_output.data[i]<0.0) ee_att_output.data[i]+=PI2;

			EEPosition[i].SetLine((float)(ee_pos_output.data[i]/2000.0));
			EEAttitude[i].SetLine((float)(DEG*ee_att_output.data[i]/2000.0));
		}
		if(update_angles) {
			arm_ee_angles=ee_att_output;
			if(!bFirstStep) update_angles=false;
		}

		if(update_vectors) {
			arm_ik_dir=RotateVectorZ( dirEE, -RMS_ROLLOUT_ANGLE );
			arm_ik_dir=ConvertVectorToRMSFrame( arm_ik_dir );

			arm_ik_rot=RotateVectorZ( rotEEik, -RMS_ROLLOUT_ANGLE );
			arm_ik_rot=ConvertVectorToRMSFrame( arm_ik_rot );

			arm_ik_pos=RotateVectorZ( posEE - RMS_SP_JOINT, -RMS_ROLLOUT_ANGLE );
			arm_ik_pos=ConvertVectorToRMSFrame( arm_ik_pos );

			if(!bFirstStep) update_vectors=false;
		}

		if(!bFirstStep) arm_moved=false;
	}

	UpdateRates( simdt );

	cameraElbow->TimeStep( simdt );
	cameraWrist->TimeStep( simdt );

	if(bFirstStep) {
		bFirstStep = false;
	}

	// handle light and talkback outputs
	// HACK most things here should be powered by the RMS Power and not the RMS Select switch
	if (Powered())
	{
		if(Eq(shoulder_brace, 0.0, 0.01)) ShoulderBraceReleased.SetLine();

		// check reach limits
		bool tmp = false;
		for (int i = SHOULDER_YAW; i <= WRIST_ROLL; i++)
		{
			if ((joint_angle[i] < RMS_JOINT_REACHLIMITS[0][i]) || (joint_angle[i] > RMS_JOINT_REACHLIMITS[1][i]))
			{
				tmp = true;
				break;
			}
		}

		if ((tmp == true) && (ReachLimit == false))
		{
			// if option enabled, slow down to x1.0 when alarm is triggered
			if (STS()->GetOptions()->Alarm1x() && (oapiGetTimeAcceleration() > 1.0)) oapiSetTimeAcceleration( 1.0 );

			SoundPlay( STS()->GetSound(), CW_TONE_RMS_SOUND, true );
			MasterAlarmOn = true;
		}
		ReachLimit = tmp;

		CWLights[9].SetLine( (int)ReachLimit * 5.0f );// reach lim light
		CWLights[11].SetLine( (int)MasterAlarmOn * 5.0f );// master alarm light

		for (int i = 0; i < 12; i++) ModeLights[i].SetLine( (int)(effMode == i) * 5.0f );
		ReadyLight.SetLine( (int)(autoState == AUTO_READY) * 5.0f );
		InProgLight.SetLine( (int)(autoState == AUTO_RUN) * 5.0f );

		SoftStopTB.SetLine( (int)bSoftStop * 5.0f );
		RateMinTB.SetLine( (int)!RateCoarse() * 5.0f );

		EECapture.SetLine( (int)bEECapture * 5.0f );
		EEExtended.SetLine( (int)bEEExtended * 5.0f );
		EEClosed.SetLine( (int)bEEClosed * 5.0f );
		EEOpened.SetLine( (int)bEEOpened * 5.0f );
		EERigidized.SetLine( (int)bEERigidized * 5.0f );
		EEDerigidized.SetLine( (int)bEEDerigidized * 5.0f );
	}
	else
	{
		ShoulderBraceReleased.ResetLine();

		ReadyLight.ResetLine();
		InProgLight.ResetLine();
		for (int i = 0; i < 12; i++)
		{
			ModeLights[i].ResetLine();
			CWLights[i].ResetLine();
		}

		SoftStopTB.ResetLine();
		RateMinTB.ResetLine();

		EECapture.ResetLine();
		EEExtended.ResetLine();
		EEClosed.ResetLine();
		EEOpened.ResetLine();
		EERigidized.ResetLine();
		EEDerigidized.ResetLine();

		SoundStop( STS()->GetSound(), CW_TONE_RMS_SOUND );
	}
}

bool RMS::OnParseLine(const char* line)
{
	if (!_strnicmp( line, "ANGLES", 6 ))
	{
		sscanf_s( line + 6, "%lf %lf %lf %lf %lf %lf",
			&joint_angle[SHOULDER_YAW], &joint_angle[SHOULDER_PITCH],
			&joint_angle[ELBOW_PITCH],
			&joint_angle[WRIST_PITCH], &joint_angle[WRIST_YAW], &joint_angle[WRIST_ROLL] );


		joint_angle[SHOULDER_YAW] = -joint_angle[SHOULDER_YAW];// HACK fix the wrong sign in the shoulder yaw joint (without touching the IK part)
		arm_moved=true;
		update_vectors=true;
		update_angles=true;
		return true;
	}
	else if (!_strnicmp( line, "ELBOW_CAM", 9 ))
	{
		cameraElbow->LoadState( line + 9 );
		return true;
	}
	else if (!_strnicmp( line, "WRIST_CAM", 9 ))
	{
		cameraWrist->LoadState( line + 9 );
		return true;
	}
	else if(!_strnicmp(line, "GRAPPLE", 7)) {
		sscan_state((char*)line+7, Grapple_State);
		return true;
	}
	else if(!_strnicmp(line, "RIGIDIZE", 8)) {
		sscan_state((char*)line+8, Rigid_State);
		return true;
	}
	else if(!_strnicmp(line, "EXTEND", 6)) {
		sscan_state((char*)line+6, Extend_State);
		return true;
	}
	else if(!_strnicmp(line, "SHOULDER_BRACE", 14)) {
		sscanf_s(line+14, "%lf", &shoulder_brace);
		return true;
	}
	else if(!_strnicmp(line, "ACCEPTED_MODE", 13)) {
		sscanf_s(line+13, "%d", &acceptedMode);
		return true;
	}
	else if(!_strnicmp(line, "PDRS_ID", 7)) {
		sscanf_s(line+7, "%d %d", &pdrs.plid, &pdrs.eeid);
		return true;
	}

	return MPM::OnParseLine(line);
}

void RMS::OnSaveState(FILEHANDLE scn) const
{
	char cbuf[256];

	sprintf_s( cbuf, 256, "%lf %lf %lf %lf %lf %lf",
		-joint_angle[SHOULDER_YAW]/*HACK fix the wrong sign in the shoulder yaw joint (without touching the IK part)*/, joint_angle[SHOULDER_PITCH],
		joint_angle[ELBOW_PITCH],
		joint_angle[WRIST_PITCH], joint_angle[WRIST_YAW], joint_angle[WRIST_ROLL] );
	oapiWriteScenario_string( scn, "ANGLES", cbuf );
	oapiWriteScenario_float(scn, "SHOULDER_BRACE", shoulder_brace);
	oapiWriteScenario_int( scn, "ACCEPTED_MODE", acceptedMode );
	sprintf_s( cbuf, 256, "%d %d", pdrs.plid, pdrs.eeid );
	oapiWriteScenario_string( scn, "PDRS_ID", cbuf );
	WriteScenario_state(scn, "GRAPPLE", Grapple_State);
	WriteScenario_state(scn, "RIGIDIZE", Rigid_State);
	WriteScenario_state(scn, "EXTEND", Extend_State);
	cameraElbow->SaveState( cbuf );
	oapiWriteScenario_string( scn, "ELBOW_CAM", cbuf );
	cameraWrist->SaveState( cbuf );
	oapiWriteScenario_string( scn, "WRIST_CAM", cbuf );

	MPM::OnSaveState(scn);
	return;
}

void RMS::VisualCreated( VISHANDLE vis )
{
	// update UV in light
	light->VisualCreated();

	GROUPEDITSPEC grpSpec;
	grpSpec.flags = GRPEDIT_SETUSERFLAG;
	grpSpec.UsrFlag = 0x00000003;// hide group and shadow

	// update UV in camera lights
	CCTVCameraPTU_LED* ledptu = dynamic_cast<CCTVCameraPTU_LED*>(cameraElbow);
	if (ledptu)
	{
		ledptu->VisualCreated();

		// hide -506/-508
		oapiEditMeshGroup( STS()->GetDevMesh( STS()->Get_vis(), mesh_index_RMS ), GRP_ELBOW_CAMERA_MINUS506MINUS508_RMS_Port, &grpSpec );
		oapiEditMeshGroup( STS()->GetDevMesh( STS()->Get_vis(), mesh_index_RMS ), GRP_ELBOW_CAMERA_PTU_GOLD_RMS_Port, &grpSpec );
		oapiEditMeshGroup( STS()->GetDevMesh( STS()->Get_vis(), mesh_index_RMS ), GRP_ELBOW_CAMERA_PTU_BASE_GOLD_RMS_Port, &grpSpec );
	}
	else
	{
		// not using illuminator, hide those parts
		oapiEditMeshGroup( STS()->GetDevMesh( STS()->Get_vis(), mesh_index_RMS ), GRP_ELBOW_CAMERA_LED_CABLE_1_RMS_Port, &grpSpec );
		oapiEditMeshGroup( STS()->GetDevMesh( STS()->Get_vis(), mesh_index_RMS ), GRP_ELBOW_CAMERA_LED_CABLE_2_RMS_Port, &grpSpec );
		oapiEditMeshGroup( STS()->GetDevMesh( STS()->Get_vis(), mesh_index_RMS ), GRP_ELBOW_CAMERA_LED_ILLUMINATOR_BODY_RMS_Port, &grpSpec );
		oapiEditMeshGroup( STS()->GetDevMesh( STS()->Get_vis(), mesh_index_RMS ), GRP_ELBOW_CAMERA_LED_ILLUMINATOR_FRONT_RMS_Port, &grpSpec );
		oapiEditMeshGroup( STS()->GetDevMesh( STS()->Get_vis(), mesh_index_RMS ), GRP_ELBOW_CAMERA_LED_POWER_BOX_RMS_Port, &grpSpec );

		// if CTVC/ITVC, hide -506/-508
		if (missionrms.Elbow == 1)
		{
			oapiEditMeshGroup( STS()->GetDevMesh( STS()->Get_vis(), mesh_index_RMS ), GRP_ELBOW_CAMERA_MINUS506MINUS508_RMS_Port, &grpSpec );
			oapiEditMeshGroup( STS()->GetDevMesh( STS()->Get_vis(), mesh_index_RMS ), GRP_ELBOW_CAMERA_PTU_GOLD_RMS_Port, &grpSpec );
			oapiEditMeshGroup( STS()->GetDevMesh( STS()->Get_vis(), mesh_index_RMS ), GRP_ELBOW_CAMERA_PTU_BASE_GOLD_RMS_Port, &grpSpec );
		}
		else// else, hide CTVC/ITVC
		{
			oapiEditMeshGroup( STS()->GetDevMesh( STS()->Get_vis(), mesh_index_RMS ), GRP_ELBOW_CAMERA_CTVCITVC_RMS_Port, &grpSpec );
			oapiEditMeshGroup( STS()->GetDevMesh( STS()->Get_vis(), mesh_index_RMS ), GRP_ELBOW_CAMERA_PTU_SILVER_RMS_Port, &grpSpec );
			oapiEditMeshGroup( STS()->GetDevMesh( STS()->Get_vis(), mesh_index_RMS ), GRP_ELBOW_CAMERA_PTU_BASE_SILVER_RMS_Port, &grpSpec );
		}
	}

	CCTVCamera_LED* led = dynamic_cast<CCTVCamera_LED*>(cameraWrist);
	if (led)
	{
		led->VisualCreated();

		// hide -506/-508
		oapiEditMeshGroup( STS()->GetDevMesh( STS()->Get_vis(), mesh_index_RMS ), GRP_WRIST_CAMERA_MINUS506MINUS508_RMS_Port, &grpSpec );
	}
	else
	{
		// not using illuminator, hide those parts
		oapiEditMeshGroup( STS()->GetDevMesh( STS()->Get_vis(), mesh_index_RMS ), GRP_WRIST_CAMERA_LED_CABLE_1_RMS_Port, &grpSpec );
		oapiEditMeshGroup( STS()->GetDevMesh( STS()->Get_vis(), mesh_index_RMS ), GRP_WRIST_CAMERA_LED_CABLE_2_RMS_Port, &grpSpec );
		oapiEditMeshGroup( STS()->GetDevMesh( STS()->Get_vis(), mesh_index_RMS ), GRP_WRIST_CAMERA_LED_ILLUMINATOR_BODY_RMS_Port, &grpSpec );
		oapiEditMeshGroup( STS()->GetDevMesh( STS()->Get_vis(), mesh_index_RMS ), GRP_WRIST_CAMERA_LED_ILLUMINATOR_FRONT_RMS_Port, &grpSpec );
		oapiEditMeshGroup( STS()->GetDevMesh( STS()->Get_vis(), mesh_index_RMS ), GRP_WRIST_CAMERA_LED_POWER_BOX_RMS_Port, &grpSpec );

		// if CTVC/ITVC, hide -506/-508
		if (missionrms.Wrist == 1)
		{
			oapiEditMeshGroup( STS()->GetDevMesh( STS()->Get_vis(), mesh_index_RMS ), GRP_WRIST_CAMERA_MINUS506MINUS508_RMS_Port, &grpSpec );
		}
		else// else, hide CTVC/ITVC
		{
			oapiEditMeshGroup( STS()->GetDevMesh( STS()->Get_vis(), mesh_index_RMS ), GRP_WRIST_CAMERA_CTVCITVC_RMS_Port, &grpSpec );
		}
	}
	return;
}

void RMS::CreateCCTV( void )
{
	if (missionrms.Elbow == 0)// -506/-508
	{
		cameraElbow = new CCTVCameraPTU( STS(), RMS_ELBOW_PTU_POS );
	}
	else// if (missionrms.Elbow == 1)// CTVC/ITVC
	{
		if (missionrms.ElbowIlluminator) cameraElbow = new CCTVCameraPTU_LED( STS(), RMS_ELBOW_PTU_POS );
		else cameraElbow = new CCTVCameraPTU( STS(), RMS_ELBOW_PTU_POS );
	}

	if (missionrms.Wrist == 0)// -506/-508
	{
		cameraWrist = new CCTVCamera( STS(), RMS_EE_CAM_POS );
	}
	else// if (missionrms.Wrist == 1)// CTVC/ITVC
	{
		if (missionrms.WristIlluminator) cameraWrist = new CCTVCamera_LED( STS(), RMS_EE_CAM_POS );
		else cameraWrist = new CCTVCamera( STS(), RMS_EE_CAM_POS );
	}

	videoswitcher = new RemoteVideoSwitcher( portside, cameraElbow, cameraWrist, STS() );
	return;
}

void RMS::LoadAutoSequences( void )
{
	for (int i = 0; i < 20; i++) autoseq[i].clear();

	std::ifstream file( ".\\Config\\SSV\\RMS\\AutoSequences.txt" );
	if (!file.is_open())
	{
		oapiWriteLog( "(SSV_OV) [INFO] RMS: Config\\SSV\\RMS\\AutoSequences.txt not found, no auto sequences loaded" );
		return;
	}

	// format: "SEQ n" starts sequence n (1 to 20); "PT X Y Z P Y R [PAUSE]" adds a point (BACS position in inches, attitude in degrees); "#" starts a comment
	std::string line;
	int cur = -1;
	while (std::getline( file, line ))
	{
		size_t comment = line.find( '#' );
		if (comment != std::string::npos) line.erase( comment );

		std::istringstream iss( line );
		std::string key;
		if (!(iss >> key)) continue;

		if (!_stricmp( key.c_str(), "SEQ" ))
		{
			int id;
			if ((iss >> id) && (id >= 1) && (id <= 20)) cur = id - 1;
			else cur = -1;
		}
		else if (!_stricmp( key.c_str(), "PT" ) && (cur >= 0))
		{
			AutoPoint pt;
			pt.pause = false;
			if (iss >> pt.pos[0] >> pt.pos[1] >> pt.pos[2] >> pt.att[0] >> pt.att[1] >> pt.att[2])
			{
				std::string flag;
				if ((iss >> flag) && !_stricmp( flag.c_str(), "PAUSE" )) pt.pause = true;
				if (autoseq[cur].size() < 200) autoseq[cur].push_back( pt );
			}
		}
	}

	int count = 0;
	for (int i = 0; i < 20; i++) if (!autoseq[i].empty()) count++;
	oapiWriteLogV( "(SSV_OV) [INFO] RMS: %d auto sequence(s) loaded", count );
	return;
}

int RMS::GetAutoSequenceSize( int id ) const
{
	if ((id < 1) || (id > 20)) return 0;
	return (int)autoseq[id - 1].size();
}

void RMS::GetEEState( VECTOR3& pos, MATRIX3& att ) const
{
	// position, same calculation as for the LED displays: BACS, in inches
	VECTOR3 ee = (posEE - RMS_SP_JOINT) / IN2M;
	pos = _V( ee.z, ee.x, -ee.y ) + _V( -679.5, -108.0, -444.77 );

	// attitude: reference frame defined by the EE direction, in the Orbiter body axes
	VECTOR3 x = ConvertVectorToRMSFrame( dirEE );
	VECTOR3 z = ConvertVectorToRMSFrame( -rotEE );
	VECTOR3 y = crossp( z, x );
	att = _M( x.x, y.x, z.x,
			  x.y, y.y, z.y,
			  x.z, y.z, z.z );
	return;
}

bool RMS::GetAutoPOR( VECTOR3& porOffset ) const
{
	porOffset = _V( 0.0, 0.0, 0.0 );
	if (pdrs.plid <= 0) return false;
	VECTOR3 axes[3];
	return GetPayloadPOR( porOffset, axes );
}

void RMS::GetPORError( const double tgtpos[3], const double tgtatt[3], VECTOR3& dB, VECTOR3& axis, double& angle ) const
{
	VECTOR3 cur;
	MATRIX3 curM;
	GetEEState( cur, curM );

	VECTOR3 porOffset;
	GetAutoPOR( porOffset );// zero for the EE tip
	RMSMath::PORError( cur, curM, porOffset, tgtpos, tgtatt, dB, axis, angle );
	return;
}

bool RMS::CheckOPRCMD( void ) const
{
	for (int i = 0; i < 3; i++)
	{
		if (!pdrs.endpos_valid[i] || !pdrs.endatt_valid[i]) return false;
	}

	VECTOR3 dB, axis;
	double angle;
	GetPORError( pdrs.endpos, pdrs.endatt, dB, axis, angle );

	VECTOR3 porOffset;
	bool usepor = GetAutoPOR( porOffset );

	// rotation matrix for the whole attitude error
	MATRIX3 R = RMSMath::MatrixFromAxisAngle( axis, angle );

	VECTOR3 newDir = RotateVectorX( mul( R, RotateVectorX( arm_ik_dir, -RMS_ROLLOUT_ANGLE ) ), RMS_ROLLOUT_ANGLE );
	VECTOR3 newRot = RotateVectorX( mul( R, RotateVectorX( arm_ik_rot, -RMS_ROLLOUT_ANGLE ) ), RMS_ROLLOUT_ANGLE );
	VECTOR3 newPos = arm_ik_pos + RotateVectorX( dB, RMS_ROLLOUT_ANGLE );
	if (usepor) newPos += RotateVectorX( porOffset - mul( R, porOffset ), RMS_ROLLOUT_ANGLE );

	double angles[6];
	if (!ComputeJointAngles( newPos, newDir, newRot, angles )) return false;
	for (int i = SHOULDER_YAW; i <= WRIST_ROLL; i++)
	{
		if ((angles[i] < RMS_JOINT_REACHLIMITS[0][i]) || (angles[i] > RMS_JOINT_REACHLIMITS[1][i])) return false;
	}
	return true;
}

bool RMS::GetAutoTarget( double tgtpos[3], double tgtatt[3], bool& pause ) const
{
	pause = false;
	if (ModeIs( MODE_OPR_CMD ))
	{
		for (int i = 0; i < 3; i++)
		{
			tgtpos[i] = pdrs.endpos[i];
			tgtatt[i] = pdrs.endatt[i];
		}
		return true;
	}

	int idx = effMode - MODE_AUTO_1;
	if ((idx < 0) || (idx > 3)) return false;
	const std::vector<AutoPoint>& seq = autoseq[pdrs.autoseq[idx] - 1];
	int next = autoPt + 1;// the points are numbered from 1
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

void RMS::RunAutoModes( double simdt, bool armfree )
{
	const bool automode = ModeIs( MODE_OPR_CMD ) || ModeIs( MODE_AUTO_1 ) || ModeIs( MODE_AUTO_2 ) || ModeIs( MODE_AUTO_3 ) || ModeIs( MODE_AUTO_4 );
	const bool proceed = ovr.auto_ena ? ovr.auto_proceed : AutoSeqProceed.IsSet();

	if (!automode)
	{
		autoState = AUTO_IDLE;
		bAutoProceedPrev = proceed;
		return;
	}

	if (!bAutoModeLogged)
	{
		// mode just entered
		bAutoModeLogged = true;
		autoState = AUTO_IDLE;
		if (!ModeIs( MODE_OPR_CMD ))
		{
			// AUTO 1-4: the arm must be at the start point of the sequence selected with the items 13-16 of the PDRS CONTROL display
			int idx = effMode - MODE_AUTO_1;
			const std::vector<AutoPoint>& seq = autoseq[pdrs.autoseq[idx] - 1];
			int start = (pdrs.startpt > 0) ? pdrs.startpt : 1;
			if (((int)seq.size() >= 2) && (start < (int)seq.size()))
			{
				autoPt = start;
				pdrs.lastpt = start;
				autoState = AUTO_READY;
			}
			else oapiWriteLogV( "(SSV_OV) [INFO] RMS: auto sequence %d has no points to fly (see Config\\SSV\\RMS\\AutoSequences.txt)", pdrs.autoseq[idx] );
		}
	}

	if (ModeIs( MODE_OPR_CMD ) && (autoState != AUTO_RUN))
	{
		// OPR CMD is ready when the command check (item 25) is GOOD and PL INIT ID (item 24) matches PL ID (item 3)
		autoState = ((pdrs.cmdck == 1) && (pdrs.plinitid == pdrs.plid)) ? AUTO_READY : AUTO_IDLE;
	}

	// AUTO SEQ switch: PROCEED starts or continues the sequence, STOP brings the arm to rest
	if (proceed && !bAutoProceedPrev && (autoState == AUTO_READY) && armfree) autoState = AUTO_RUN;
	else if (!proceed && (autoState == AUTO_RUN)) autoState = AUTO_READY;
	bAutoProceedPrev = proceed;

	if (autoState != AUTO_RUN) return;
	if (!armfree || bSoftStop)
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
			pdrs.cmdck = 0;
			autoState = AUTO_IDLE;
		}
		else
		{
			const int idx = effMode - MODE_AUTO_1;
			const int size = (int)autoseq[pdrs.autoseq[idx] - 1].size();
			autoPt++;
			pdrs.lastpt = autoPt;
			if (autoPt >= size) autoState = AUTO_IDLE;// end of the sequence
			else if (pause) autoState = AUTO_READY;// programmed pause, PROCEED must be selected again
		}
		return;
	}

	const double vt = (RateCoarse() ? RMS_EE_COARSE_TRANSLATION_SPEED : RMS_EE_VERN_TRANSLATION_SPEED) * GetLoadScale();
	const double vr = (RateCoarse() ? RMS_EE_COARSE_ROTATION_SPEED : RMS_EE_VERN_ROTATION_SPEED) * GetLoadScale();

	VECTOR3 newPos = arm_ik_pos;
	VECTOR3 newDir = arm_ik_dir;
	VECTOR3 newRot = arm_ik_rot;

	if (angle > 1e-6)
	{
		VECTOR3 porOffset;
		bool usepor = GetAutoPOR( porOffset );
		const VECTOR3 axes[3] = {_V( 1.0, 0.0, 0.0 ), _V( 0.0, 1.0, 0.0 ), _V( 0.0, 0.0, 1.0 )};
		double w = min( vr, max( angle, 0.002 ) );
		double step = min( angle, w * simdt );
		RotateAboutPOR( axis * step, axes, usepor, porOffset, newPos, newDir, newRot );
		update_angles = true;
		rotRateCmd = axis * (step * DEG / simdt);
	}
	if (dist > 1e-6)
	{
		double v = min( vt, max( dist, 0.002 ) );
		double step = min( dist, v * simdt );
		newPos += RotateVectorX( (dB / dist) * step, RMS_ROLLOUT_ANGLE );
		transRateCmd = (dB / dist) * (step / (simdt * 0.3048));
	}

	if (!MoveEE( newPos, newDir, newRot, simdt ))
	{
		oapiWriteLog( "(SSV_OV) [INFO] RMS: auto sequence stopped, the arm can not reach the next position" );
		autoState = AUTO_READY;
	}
	return;
}

void RMS::RotateAboutPOR( const VECTOR3& angles, const VECTOR3 axes[3], bool usePOR, const VECTOR3& porOffset, VECTOR3& newPos, VECTOR3& newDir, VECTOR3& newRot )
{
	RMSMath::RotateAboutPOR( angles, axes, usePOR, porOffset, RMS_ROLLOUT_ANGLE, arm_ik_dir, arm_ik_rot, newPos, newDir, newRot );
	return;
}

void RMS::LogPOR( void ) const
{
	VECTOR3 pos;
	MATRIX3 att;
	GetEEState( pos, att );
	VECTOR3 angles = GetYZX_PYRAnglesFromMatrix( att );
	for (int i = 0; i < 3; i++)
	{
		angles.data[i] *= DEG;
		if (angles.data[i] < 0.0) angles.data[i] += 360.0;
	}
	oapiWriteLogV( "(SSV_OV) [INFO] RMS point: PT %.1f %.1f %.1f %.1f %.1f %.1f", pos.x, pos.y, pos.z, angles.data[PITCH], angles.data[YAW], angles.data[ROLL] );
	return;
}

bool RMS::PDRSItemInput( int item, const char* data )
{
	int n;
	double d;
	const bool nodata = (strlen( data ) == 0);

	switch (item)
	{
		case 3:// PL ID
			if (sscanf_s( data, "%d", &n ) != 1 || (n < 0) || (n > 5)) return false;
			pdrs.plid = n;
			pdrs.cmdck = 0;
			return true;
		case 4:// END EFF ID
			if (sscanf_s( data, "%d", &n ) != 1 || (n < 1) || (n > 2)) return false;
			pdrs.eeid = n;
			return true;
		case 5:
		case 6:// I/O ON/OFF
			if (!nodata) return false;
			pdrs.io = (item == 5);
			return true;
		case 7:
		case 8:// SOFT STOP ENA/INH
			if (!nodata) return false;
			pdrs.softstop_ena = (item == 7);
			return true;
		case 9:
		case 10:// AUTO BRAKE ENA/INH
			if (!nodata) return false;
			pdrs.autobrake_ena = (item == 9);
			return true;
		case 11:
		case 12:// POS ENC CK ENA/INH
			if (!nodata) return false;
			pdrs.posenc_ena = (item == 11);
			return true;
		case 13:
		case 14:
		case 15:
		case 16:// AUTO 1-4 sequence ID
			if (sscanf_s( data, "%d", &n ) != 1 || (n < 1) || (n > 20)) return false;
			pdrs.autoseq[item - 13] = n;
			return true;
		case 17:// START PT
			if (sscanf_s( data, "%d", &n ) != 1 || (n < 1) || (n > 199)) return false;
			pdrs.startpt = n;
			return true;
		case 18:
		case 19:
		case 20:// OPR CMD END POS X, Y, Z
			if (sscanf_s( data, "%lf", &d ) != 1 || (fabs( d ) > 5000.0)) return false;
			pdrs.endpos[item - 18] = d;
			pdrs.endpos_valid[item - 18] = true;
			pdrs.cmdck = 0;
			return true;
		case 21:
		case 23:// OPR CMD END ATT P, R
			if (sscanf_s( data, "%lf", &d ) != 1 || (d < 0.0) || (d >= 360.0)) return false;
			pdrs.endatt[(item == 21) ? 0 : 2] = d;
			pdrs.endatt_valid[(item == 21) ? 0 : 2] = true;
			pdrs.cmdck = 0;
			return true;
		case 22:// OPR CMD END ATT Y
			if (sscanf_s( data, "%lf", &d ) != 1 || (d < 0.0) || (d >= 360.0) || ((d > 90.0) && (d < 270.0))) return false;
			pdrs.endatt[1] = d;
			pdrs.endatt_valid[1] = true;
			pdrs.cmdck = 0;
			return true;
		case 24:// PL INIT ID
			if (sscanf_s( data, "%d", &n ) != 1 || (n < 0) || (n > 5)) return false;
			pdrs.plinitid = n;
			pdrs.cmdck = 0;
			return true;
		case 25:// CMD CK
			if (!nodata) return false;
			pdrs.cmdck = CheckOPRCMD() ? 1 : 2;
			return true;
		case 26:// WR RANGE
			if (sscanf_s( data, "%d", &n ) != 1 || (n < 1) || (n > 6)) return false;
			pdrs.wrrange = n;
			return true;
		case 27:// AUTO BRAKE CK
		case 30:// POR DISPLAY
			return nodata;
		case 28:
		case 29:// POHS CNTL ENA/INH
			if (!nodata) return false;
			pdrs.pohs_ena = (item == 28);
			return true;
		case 31:
		case 32:
		case 33:// ORB LD CMD REF NORM/FLY-CAMR/FLY-PL
			if (!nodata) return false;
			pdrs.orbldref = item - 31;
			return true;
		case 34:// RMS S/N
			return (sscanf_s( data, "%d", &n ) == 1);
		default:
			return false;
	}
}

bool RMS::Powered( void ) const
{
	return RMSSelect.IsSet() && !PowerOff.IsSet();
}

bool RMS::RateCoarse( void ) const
{
	// the RATE switch of the RHC, or the override of the PDRS OVERRIDE display
	return ovr.rate_ena ? ovr.rate_coarse : RMSCoarseRateCMD.IsSet();
}

double RMS::GetLoadScale( void ) const
{
	// the joint rates are reduced as the mass of the captured payload increases (up to 29500 kg); the unloaded arm is the reference
	// the effect of the mass is an option of the vessel configuration (RMSPayloadMassEffect, off by default)
	if (attachedPayload == NULL) return 1.0;
	// LOADED RATE override: the loaded rate limits are in effect even for light payloads
	return RMSMath::LoadScale( attachedPayload->GetMass(), STS()->GetOptions()->RMSPayloadMassEffect(), ovr.loaded_rate );
}

void RMS::UpdateRates( double dt )
{
	VECTOR3 pos;
	MATRIX3 att;
	GetEEState( pos, att );

	if (!ratesvalid || (dt <= 0.0))
	{
		lastPos = pos;
		lastAtt = att;
		ratesvalid = true;
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
	const double k = min( 1.0, dt / 0.25 );
	transRateAct = transRateAct + ((vel - transRateAct) * k);
	rotRateAct = rotRateAct + ((w - rotRateAct) * k);

	lastPos = pos;
	lastAtt = att;
	return;
}

void RMS::GetPDRSStatus( PDRSStatus& st ) const
{
	st.selected = RMSSelect.IsSet();
	st.power = PowerOff.IsSet() ? 0 : ((PowerBackup.IsSet() && !PowerPrimary.IsSet()) ? 2 : 1);
	for (int i = 0; i < 6; i++)
	{
		st.jointangle[i] = joint_angle[i];
		st.reachlimit[i] = 0;
		if (joint_angle[i] < RMS_JOINT_REACHLIMITS[0][i]) st.reachlimit[i] = -1;
		else if (joint_angle[i] > RMS_JOINT_REACHLIMITS[1][i]) st.reachlimit[i] = 1;
	}
	st.jointangle[SHOULDER_YAW] = -st.jointangle[SHOULDER_YAW];// HACK the sign of the shoulder yaw joint is inverted (see CalcVectors)

	VECTOR3 pos;
	MATRIX3 att;
	GetEEState( pos, att );
	VECTOR3 angles = GetYZX_PYRAnglesFromMatrix( att );
	for (int i = 0; i < 3; i++)
	{
		st.pos[i] = pos.data[i];
		double a = angles.data[i] * DEG;
		if (a < 0.0) a += 360.0;
		st.att[i] = a;
	}

	// rates: the Orbiter body axes (tail, port, down) are converted to BACS (forward, starboard, down); rotations are pitch, yaw and roll
	st.trans_act[0] = -transRateAct.x;
	st.trans_act[1] = -transRateAct.y;
	st.trans_act[2] = transRateAct.z;
	st.trans_cmd[0] = -transRateCmd.x;
	st.trans_cmd[1] = -transRateCmd.y;
	st.trans_cmd[2] = transRateCmd.z;
	st.rot_act[PITCH] = rotRateAct.y;
	st.rot_act[YAW] = rotRateAct.z;
	st.rot_act[ROLL] = rotRateAct.x;
	st.rot_cmd[PITCH] = rotRateCmd.y;
	st.rot_cmd[YAW] = rotRateCmd.z;
	st.rot_cmd[ROLL] = rotRateCmd.x;

	st.ee_rigid = bEERigidized;
	st.ee_close = bEEClosed;
	st.ee_capture = bEECapture;
	st.ee_derigid = bEEDerigidized;
	st.ee_open = bEEOpened;
	st.ee_extend = bEEExtended;
	return;
}

bool RMS::GetGrappleHint( double& distance, double& angle ) const
{
	// the grapple fixture that is in the field of view of the wrist camera (34 degrees), nearest to its axis; distance is along the axis
	if (!Powered() || (attachedPayload != NULL)) return false;
	return GetViewedAttachment( 17.0 * RAD, distance, angle );
}

bool RMS::PDRSOverrideItem( int item, const char* data )
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


bool RMS::ManualAugmentedMode() const
{
	return ModeIs(MODE_ORB_UNL) || ModeIs(MODE_END_EFF) || ModeIs(MODE_ORB_LD) || ModeIs(MODE_PL);
}

bool RMS::GetPayloadPOR( VECTOR3& porOffset, VECTOR3 axes[3] ) const
{
	if (attachedPayload == NULL) return false;

	// position of the payload origin in Orbiter local coordinates
	MATRIX3 Rs, Rp;
	STS()->GetRotationMatrix( Rs );
	attachedPayload->GetRotationMatrix( Rp );
	VECTOR3 payload_gpos, orbiter_gpos;
	attachedPayload->GetGlobalPos( payload_gpos );
	STS()->GetGlobalPos( orbiter_gpos );
	VECTOR3 payload_pos = tmul( Rs, payload_gpos - orbiter_gpos );

	// position of the EE tip in Orbiter local coordinates
	VECTOR3 ee_pos = STS()->GetOrbiterCoGOffset() + posEE + RMS_MESH_OFFSET;

	porOffset = ConvertVectorToRMSFrame( payload_pos - ee_pos );

	// payload axes (tail, port and down, as in the Orbiter body axes), from payload to Orbiter local coordinates and then to body axes
	const VECTOR3 payload_axes[3] = {_V( 0.0, 0.0, -1.0 ), _V( -1.0, 0.0, 0.0 ), _V( 0.0, -1.0, 0.0 )};
	for (int i = 0; i < 3; i++) axes[i] = ConvertVectorToRMSFrame( tmul( Rs, mul( Rp, payload_axes[i] ) ) );
	return true;
}

void RMS::DriveSingleJoint( int joint, int dir, double simdt )
{
	// SINGLE and DIRECT drive the joint at a fixed rate, selected by the RATE switch
	const double rate = ((RateCoarse() && !backupDrive) ? RMS_JOINT_COARSE_ROTATION_SPEEDS[joint] : RMS_JOINT_VERN_ROTATION_SPEEDS[joint]) * GetLoadScale();
	// HACK the sign of the shoulder yaw joint is inverted (see CalcVectors)
	const double sign = (joint == SHOULDER_YAW) ? -dir : dir;
	double newangle = joint_angle[joint] + (sign * rate * simdt);

	if (ModeIs(MODE_SINGLE))
	{
		// in SINGLE mode the GPC stops the joint at the soft stop and drops to idle, SINGLE must be selected again;
		// soft stops are disabled for a joint that is in a soft stop when SINGLE is selected
		if (bSingleSoftStopIdle) return;
		if (!singleSoftStopDisabled[joint])
		{
			if (newangle < RMS_JOINT_SOFTSTOPS[0][joint])
			{
				newangle = RMS_JOINT_SOFTSTOPS[0][joint];
				bSingleSoftStopIdle = true;
			}
			else if (newangle > RMS_JOINT_SOFTSTOPS[1][joint])
			{
				newangle = RMS_JOINT_SOFTSTOPS[1][joint];
				bSingleSoftStopIdle = true;
			}
		}
	}
	// DIRECT bypasses the GPC, so there are no soft stops

	SetJointAngle( (RMS_JOINT)joint, newangle );
	return;
}

void RMS::Translate(const VECTOR3 &dPos, VECTOR3& newPos)
{
	if (ModeIs(MODE_END_EFF))
	{
		// Reference Frame:
		// X: in direction of EE Z: opposite to camera direction Y: completes RH frame
		VECTOR3 y_axis = crossp(arm_ik_rot, arm_ik_dir);
		// create rotation matrix to convert vector from EE frame to Orbiter body frame
		MATRIX3 EERotMatrix = _M(arm_ik_dir.x, y_axis.x, arm_ik_rot.x,
							   arm_ik_dir.y, y_axis.y, arm_ik_rot.y,
							   arm_ik_dir.z, y_axis.z, arm_ik_rot.z);
		// matrix to convert vector from EE frame to IK frame
		MATRIX3 IKRotMatrix = mul(EERotMatrix, Transpose(GetRotationMatrix(_V(1, 0, 0), RMS_Z_AXIS_ANGLE)));
		newPos += mul(IKRotMatrix, dPos);
	}
	else if (ModeIs(MODE_ORB_UNL) || ModeIs(MODE_ORB_LD) || ModeIs(MODE_PL))
	{
		VECTOR3 dPosB = dPos;// translation in Orbiter body axes
		VECTOR3 porOffset;
		VECTOR3 axes[3];
		if (ModeIs(MODE_PL) && (pdrs.plid > 0) && GetPayloadPOR( porOffset, axes ))
		{
			// PL mode translates along the payload axes
			dPosB = (axes[0] * dPos.x) + (axes[1] * dPos.y) + (axes[2] * dPos.z);
		}
		newPos += RotateVectorX(dPosB, RMS_ROLLOUT_ANGLE);
	}
	return;
}

void RMS::Rotate(const VECTOR3 &dAngles, VECTOR3& newPos, VECTOR3& newDir, VECTOR3& newRot)
{
	if (ModeIs(MODE_END_EFF))
	{
		// NOTE: EE mode rotates relative to camera orientation
		// we do not need to compensate for angle with RMS and shuttle frames
		// in EE mode, Z-axis is in opposited direction to arm_ee_rot
		VECTOR3 y_axis = crossp(arm_ik_rot, arm_ik_dir);
		// create rotation matrix corresponding to current orientation
		MATRIX3 RotMatrix = _M(arm_ik_dir.x, y_axis.x, arm_ik_rot.x,
							   arm_ik_dir.y, y_axis.y, arm_ik_rot.y,
							   arm_ik_dir.z, y_axis.z, arm_ik_rot.z);
		// convert rotation matrix from IK frame to EE frame
		RotMatrix = mul(RotMatrix, GetRotationMatrix(_V(1, 0, 0), -RMS_Z_AXIS_ANGLE));
		// update rotation matrix to adjust for new angles
		MATRIX3 RotMatrixRoll, RotMatrixPitch, RotMatrixYaw;
		GetRotMatrixX(dAngles.data[ROLL], RotMatrixRoll);
		GetRotMatrixY(dAngles.data[PITCH], RotMatrixPitch);
		GetRotMatrixZ(dAngles.data[YAW], RotMatrixYaw);
		RotMatrix = mul(RotMatrix, RotMatrixPitch);
		RotMatrix = mul(RotMatrix, RotMatrixYaw);
		RotMatrix = mul(RotMatrix, RotMatrixRoll);
		// convert rotation matrix from EE frame to IK frame
		RotMatrix = mul(RotMatrix, GetRotationMatrix(_V(1, 0, 0), RMS_Z_AXIS_ANGLE));

		newDir = _V(RotMatrix.m11, RotMatrix.m21, RotMatrix.m31);
		newRot = _V(RotMatrix.m13, RotMatrix.m23, RotMatrix.m33);

		update_angles=true;
	}
	else if (ModeIs(MODE_ORB_UNL) || ModeIs(MODE_ORB_LD) || ModeIs(MODE_PL))
	{
		// ORB UNL rotates about the EE tip; ORB LD and PL rotate about the POR of the payload selected with PL ID
		// (the Orbiter body axes for ORB LD, the payload axes for PL)
		VECTOR3 porOffset = _V( 0.0, 0.0, 0.0 );
		VECTOR3 axes[3];
		bool payload = false;
		if ((ModeIs(MODE_ORB_LD) || ModeIs(MODE_PL)) && (pdrs.plid > 0)) payload = GetPayloadPOR( porOffset, axes );
		if (!payload || !ModeIs(MODE_PL))
		{
			axes[0] = _V( 1.0, 0.0, 0.0 );
			axes[1] = _V( 0.0, 1.0, 0.0 );
			axes[2] = _V( 0.0, 0.0, 1.0 );
		}
		RotateAboutPOR( _V( dAngles.data[ROLL], dAngles.data[PITCH], dAngles.data[YAW] ), axes, payload, porOffset, newPos, newDir, newRot );
		update_angles=true;
	}
	return;
}

bool RMS::ComputeJointAngles(const VECTOR3 &newPos, const VECTOR3 &newDir, const VECTOR3 &newRot, double new_joint_angles[6]) const
{
	// NOTE: for cross products, |a x b|=|a||b|sin(theta)
	// all vectors obtained as cross products should be normalized
	// new_joint_angles in degrees

	VECTOR3 arm_wy_cpos=newPos-newDir*RMS_WY_EE_DIST;
	double yaw_angle=atan2(arm_wy_cpos.y, arm_wy_cpos.x);

	VECTOR3 boom_plane=_V(cos(yaw_angle), sin(yaw_angle), 0.0);
	//find vector in XY plane and perpendicular to arm booms
	VECTOR3 normal=crossp(boom_plane, _V(0.0, 0.0, 1.0));
	normal/=length(normal);

	if((newDir.z>normal.z && newDir.z<boom_plane.z) || (newDir.z<normal.z && newDir.z>boom_plane.z))
		new_joint_angles[WRIST_YAW] = -DEG*acos(dotp(normal, newDir))+90.0;
	else new_joint_angles[WRIST_YAW] = -90.0+DEG*acos(dotp(normal, newDir));

	double phi;
	//find vector in same plane as arm booms and perpendicular to EE direction
	VECTOR3 wp_normal=crossp(normal, newDir);
	if(Eq(length(wp_normal), 0.0, 0.001)) //check if newAtt is perpendicular to arm booms
	{
		// use same phi angle as for previous position
		phi=joint_angle[SHOULDER_PITCH]+joint_angle[ELBOW_PITCH]+joint_angle[WRIST_PITCH];
		wp_normal=RotateVectorZ(boom_plane, phi+90.0);
	}
	else {
		wp_normal/=length(wp_normal);
		phi=DEG*acos(wp_normal.z);
		if(newDir.z>0.0) phi=-phi;
	}

	new_joint_angles[WRIST_ROLL]=-acos(dotp(wp_normal, newRot))*DEG;

	VECTOR3 cross_product=crossp(wp_normal, newRot);
	if(Eq(1.0, dotp(cross_product/length(cross_product), newDir), 0.05))
		new_joint_angles[WRIST_ROLL]=-new_joint_angles[WRIST_ROLL];

	VECTOR3 arm_wp_dir=crossp(wp_normal, normal); // wp_normal and normal vectors are perpendicular
	VECTOR3 arm_wp_cpos=arm_wy_cpos-arm_wp_dir*RMS_WP_WY_DIST;
	double r=length(arm_wp_cpos);

	new_joint_angles[SHOULDER_YAW] = -DEG*yaw_angle;
	double rho=sqrt(arm_wp_cpos.x*arm_wp_cpos.x+arm_wp_cpos.y*arm_wp_cpos.y);
	double cos_phibar_e=(r*r-RMS_SP_EP_DIST*RMS_SP_EP_DIST-RMS_EP_WP_DIST*RMS_EP_WP_DIST)/(-2*RMS_SP_EP_DIST*RMS_EP_WP_DIST);
	if (fabs(cos_phibar_e)>1) return false;//Can't reach new point with the elbow
	new_joint_angles[ELBOW_PITCH]=DEG*acos(cos_phibar_e)-180.0-RMS_EP_NULL_ANGLE-RMS_SP_NULL_ANGLE;
	double cos_phi_s2=(RMS_EP_WP_DIST*RMS_EP_WP_DIST-RMS_SP_EP_DIST*RMS_SP_EP_DIST-r*r)/(-2*RMS_SP_EP_DIST*r);
	if(fabs(cos_phi_s2)>1) return false; //Can't reach with shoulder
	new_joint_angles[SHOULDER_PITCH]=DEG*(-atan2(arm_wp_cpos.z,rho)+acos(cos_phi_s2))+RMS_SP_NULL_ANGLE;

	new_joint_angles[WRIST_PITCH]=phi-new_joint_angles[SHOULDER_PITCH]-new_joint_angles[ELBOW_PITCH];


	// allow use of full motion range of the wrist roll joint
	if ((joint_angle[WRIST_ROLL] - new_joint_angles[WRIST_ROLL]) > 180.0)
	{
		new_joint_angles[WRIST_ROLL] += 360.0;
	}
	else if ((joint_angle[WRIST_ROLL] - new_joint_angles[WRIST_ROLL]) < -180.0)
	{
		new_joint_angles[WRIST_ROLL] -= 360.0;
	}

	return true;
}

bool RMS::MoveEE(const VECTOR3 &newPos, const VECTOR3 &newDir, const VECTOR3 &newRot, double simdt)
{
	double new_joint_angles[6]; // angles in degrees
	if (!ComputeJointAngles( newPos, newDir, newRot, new_joint_angles )) return false;

	// check values are within bounds
	// make sure speed of each joint is within limits
	bool move = true;
	for(int i=SHOULDER_YAW;i<=WRIST_ROLL;i++)
	{
		if (bSoftStop) move = false;

		double speed = abs(new_joint_angles[i]-joint_angle[i])/simdt;
		if (speed > RMS_JOINT_MAX_ROTATION_SPEED[i])
		{
			sprintf_s(oapiDebugString(), 256, "Error: joint %d reached speed limit %f", i, speed);
			move = false;
		}
	}
	if (move == false) return false;

	for(int i=SHOULDER_YAW;i<=WRIST_ROLL;i++) {
		SetJointAngle(static_cast<RMS_JOINT>(i), new_joint_angles[i]);
	}

	arm_ik_pos=newPos;
	arm_ik_dir=newDir;
	arm_ik_rot=newRot;

	return true;
}

void RMS::SetJointAngle(RMS_JOINT joint, double angle)
{
	double pos=linterp(RMS_JOINT_LIMITS[0][joint], 0.0, RMS_JOINT_LIMITS[1][joint], 1.0, angle);
	if (pos>=0.0 && pos<=1.0)
	{
		STS()->SetAnimation(anim_joint[joint], pos);
		joint_angle[joint]=angle;
		JointAngles[joint].SetLine(static_cast<float>(5.0*joint_angle[joint]/9999.0));
		arm_moved=true;
	}
}

int RMS::GetSelectedJoint() const
{
	if (ovr.joint_ena && (ovr.joint >= 0)) return ovr.joint;// JNT SW OVRD
	for(int i=0;i<6;i++) {
		if(JointSelect[i]) return i;
	}
	return -1;
}

void RMS::OnMRLLatched( void )
{
	// if RMS is in stowed position when MRL latches close, set joints to 0 and hold it
	if (ArmStowed())
	{
		for (int i = 0; i < 6; i++) SetJointAngle( (RMS_JOINT)i, 0.0 );
		stowed_and_latched = true;
	}
	return;
}

void RMS::OnMRLReleased( void )
{
	stowed_and_latched = false;
	return;
}

void RMS::OnAttach( void )
{
	MPM::OnAttach();
	bEECapture = true;
	return;
}

void RMS::OnDetach( void )
{
	MPM::OnDetach();
	bEECapture = false;
	return;
}

bool RMS::ArmStowed( void ) const
{
	if(!Eq(joint_angle[SHOULDER_YAW], 0.0, MRL_MAX_ANGLE_ERROR)) return false;
	if(!Eq(joint_angle[SHOULDER_PITCH], 0.0, MRL_MAX_ANGLE_ERROR)) return false;
	if(!Eq(joint_angle[ELBOW_PITCH], 0.0, MRL_MAX_ANGLE_ERROR)) return false;
	if(!Eq(joint_angle[WRIST_PITCH], 0.0, MRL_MAX_ANGLE_ERROR)) return false;
	if(!Eq(joint_angle[WRIST_YAW], 0.0, MRL_MAX_ANGLE_ERROR)) return false;
	if(!Eq(joint_angle[WRIST_ROLL], 0.0, MRL_MAX_ANGLE_ERROR)) return false;
	return true;
}

void RMS::UpdateEELight( void )
{
	light->UpdateLightPosition( STS()->GetOrbiterCoGOffset() + posLight + RMS_MESH_OFFSET );
	light->UpdateLightDirection( dirEE );
	return;
}

void RMS::AutoGrappleSequence()
{
	if(bAutoGrapple) {
		if(!Grapple_State.Closed()) {
			Grapple_State.action=AnimState::CLOSING;
			if(Extend_State.Moving()) Extend_State.action=AnimState::STOPPED;
			if(Rigid_State.Moving()) Rigid_State.action=AnimState::STOPPED;
		}
		else if(!Extend_State.Closed()) {
			Extend_State.action=AnimState::CLOSING;
			if(Rigid_State.Moving()) Rigid_State.action=AnimState::STOPPED;
		}
		else if(!Rigid_State.Closed()) Rigid_State.action=AnimState::CLOSING;
		else bAutoGrapple=false;
	}
}

void RMS::AutoReleaseSequence()
{
	if(bAutoRelease) {
		if(!Rigid_State.Open()) {
			Rigid_State.action=AnimState::OPENING;
			if(Grapple_State.Moving()) Grapple_State.action=AnimState::STOPPED;
			if(Extend_State.Moving()) Extend_State.action=AnimState::STOPPED;
		}
		else if(!Grapple_State.Open()) {
			Grapple_State.action=AnimState::OPENING;
			if(Extend_State.Moving()) Extend_State.action=AnimState::STOPPED;
		}
		else if(!Extend_State.Open()) Extend_State.action=AnimState::OPENING;
		else bAutoRelease=false;
	}
}

void RMS::CheckSoftwareStop( void )
{
	bSoftStop = bSingleSoftStopIdle;

	for (int i = SHOULDER_YAW; i <= WRIST_ROLL; i++)
	{
		if ((joint_angle[i] < RMS_JOINT_SOFTSTOPS[0][i]) || (joint_angle[i] > RMS_JOINT_SOFTSTOPS[1][i]))
		{
			bSoftStop = true;
			return;
		}
	}
	return;
}

void RMS::CheckRFL( void )
{
	bool fwd = false;
	bool mid = false;
	bool aft = false;

	if (Eq( joint_angle[SHOULDER_YAW], 0.0, MRL_MAX_ANGLE_ERROR ) && Eq( joint_angle[SHOULDER_PITCH], 0.0, MRL_MAX_ANGLE_ERROR ) && ((MRL[0] > 0.5) || stowed_and_latched))
	{
		fwd = true;

		if (Eq( joint_angle[ELBOW_PITCH], 0.0, MRL_MAX_ANGLE_ERROR ) && ((MRL[1] > 0.5) || stowed_and_latched))
		{
			mid = true;

			if (Eq( joint_angle[WRIST_PITCH], 0.0, MRL_MAX_ANGLE_ERROR ) && Eq( joint_angle[WRIST_YAW], 0.0, MRL_MAX_ANGLE_ERROR ) && Eq( joint_angle[WRIST_ROLL], 0.0, MRL_MAX_ANGLE_ERROR ) && ((MRL[2] > 0.5) || stowed_and_latched))
			{
				aft = true;
			}
		}
	}

	SetRFL( fwd, mid, aft );
	return;
}

void RMS::CalcVectors( void )
{
	// MPM rotation
	MATRIX3 mrotMPM = rotm( MPM_DEPLOY_AXIS_PORT, (Rollout - 1.0) * MPM_DEPLOY_ANGLE_PORT );

	VECTOR3 posSY = mul( mrotMPM, RMS_SY_JOINT - MPM_DEPLOY_REF_PORT ) + MPM_DEPLOY_REF_PORT;
	VECTOR3 axisSY = mul( mrotMPM, RMS_SY_AXIS );

	VECTOR3 posSP = mul( mrotMPM, RMS_SP_JOINT - MPM_DEPLOY_REF_PORT ) + MPM_DEPLOY_REF_PORT;
	VECTOR3 axisSP = mul( mrotMPM, RMS_SP_AXIS );

	VECTOR3 posEP = mul( mrotMPM, RMS_EP_JOINT - MPM_DEPLOY_REF_PORT ) + MPM_DEPLOY_REF_PORT;
	VECTOR3 axisEP = mul( mrotMPM, RMS_EP_AXIS );

	posCCTVElbow = mul( mrotMPM, RMS_ELBOW_PTU_POS - MPM_DEPLOY_REF_PORT ) + MPM_DEPLOY_REF_PORT;
	dirCCTVElbow = mul( mrotMPM, RMS_ELBOW_CAM_DIR );
	rotCCTVElbow = mul( mrotMPM, RMS_ELBOW_CAM_ROT );

	VECTOR3 posWP = mul( mrotMPM, RMS_WP_JOINT - MPM_DEPLOY_REF_PORT ) + MPM_DEPLOY_REF_PORT;
	VECTOR3 axisWP = mul( mrotMPM, RMS_WP_AXIS );

	VECTOR3 posWY = mul( mrotMPM, RMS_WY_JOINT - MPM_DEPLOY_REF_PORT ) + MPM_DEPLOY_REF_PORT;
	VECTOR3 axisWY = mul( mrotMPM, RMS_WY_AXIS );

	VECTOR3 posWR = mul( mrotMPM, RMS_WY_JOINT - MPM_DEPLOY_REF_PORT ) + MPM_DEPLOY_REF_PORT;
	VECTOR3 axisWR = mul( mrotMPM, RMS_WR_AXIS );

	posEE = mul( mrotMPM, RMS_EE_POS - MPM_DEPLOY_REF_PORT ) + MPM_DEPLOY_REF_PORT;
	rotEE = mul( mrotMPM, RMS_EE_ROT );
	posCCTVEE = mul( mrotMPM, RMS_EE_CAM_POS - MPM_DEPLOY_REF_PORT ) + MPM_DEPLOY_REF_PORT;
	posLight = mul( mrotMPM, RMS_EE_LIGHT_POS - MPM_DEPLOY_REF_PORT ) + MPM_DEPLOY_REF_PORT;


	// SY rotation
	MATRIX3 mrotSY = rotm( axisSY, -joint_angle[SHOULDER_YAW] * RAD );// HACK fix wrong angle sign

	posSP = mul( mrotSY, posSP - posSY ) + posSY;
	axisSP = mul( mrotSY, axisSP );

	posEP = mul( mrotSY, posEP - posSY ) + posSY;
	axisEP = mul( mrotSY, axisEP );

	posCCTVElbow = mul( mrotSY, posCCTVElbow - posSY ) + posSY;
	dirCCTVElbow = mul( mrotSY, dirCCTVElbow );
	rotCCTVElbow = mul( mrotSY, rotCCTVElbow );

	posWP = mul( mrotSY, posWP - posSY ) + posSY;
	axisWP = mul( mrotSY, axisWP );

	posWY = mul( mrotSY, posWY - posSY ) + posSY;
	axisWY = mul( mrotSY, axisWY );

	posWR = mul( mrotSY, posWR - posSY ) + posSY;
	axisWR = mul( mrotSY, axisWR );

	posEE = mul( mrotSY, posEE - posSY ) + posSY;
	rotEE = mul( mrotSY, rotEE );
	posCCTVEE = mul( mrotSY, posCCTVEE - posSY ) + posSY;
	posLight = mul( mrotSY, posLight - posSY ) + posSY;


	// SP rotation
	MATRIX3 mrotSP = rotm( axisSP, joint_angle[SHOULDER_PITCH] * RAD );

	posEP = mul( mrotSP, posEP - posSP ) + posSP;
	axisEP = mul( mrotSP, axisEP );

	posCCTVElbow = mul( mrotSP, posCCTVElbow - posSP ) + posSP;
	dirCCTVElbow = mul( mrotSP, dirCCTVElbow );
	rotCCTVElbow = mul( mrotSP, rotCCTVElbow );

	posWP = mul( mrotSP, posWP - posSP ) + posSP;
	axisWP = mul( mrotSP, axisWP );

	posWY = mul( mrotSP, posWY - posSP ) + posSP;
	axisWY = mul( mrotSP, axisWY );

	posWR = mul( mrotSP, posWR - posSP ) + posSP;
	axisWR = mul( mrotSP, axisWR );

	posEE = mul( mrotSP, posEE - posSP ) + posSP;
	rotEE = mul( mrotSP, rotEE );
	posCCTVEE = mul( mrotSP, posCCTVEE - posSP ) + posSP;
	posLight = mul( mrotSP, posLight - posSP ) + posSP;


	// EP rotation
	// CCTV
	MATRIX3 mrotEP = rotm( axisEP, joint_angle[ELBOW_PITCH] * RAD );

	posCCTVElbow = mul( mrotEP, posCCTVElbow - posEP ) + posEP;
	dirCCTVElbow = mul( mrotEP, dirCCTVElbow );
	rotCCTVElbow = mul( mrotEP, rotCCTVElbow );

	posWP = mul( mrotEP, posWP - posEP ) + posEP;
	axisWP = mul( mrotEP, axisWP );

	posWY = mul( mrotEP, posWY - posEP ) + posEP;
	axisWY = mul( mrotEP, axisWY );

	posWR = mul( mrotEP, posWR - posEP ) + posEP;
	axisWR = mul( mrotEP, axisWR );

	posEE = mul( mrotEP, posEE - posEP ) + posEP;
	rotEE = mul( mrotEP, rotEE );
	posCCTVEE = mul( mrotEP, posCCTVEE - posEP ) + posEP;
	posLight = mul( mrotEP, posLight - posEP ) + posEP;


	// WP rotation
	MATRIX3 mrotWP = rotm( axisWP, joint_angle[WRIST_PITCH] * RAD );

	posWY = mul( mrotWP, posWY - posWP ) + posWP;
	axisWY = mul( mrotWP, axisWY );

	posWR = mul( mrotWP, posWR - posWP ) + posWP;
	axisWR = mul( mrotWP, axisWR );

	posEE = mul( mrotWP, posEE - posWP ) + posWP;
	rotEE = mul( mrotWP, rotEE );
	posCCTVEE = mul( mrotWP, posCCTVEE - posWP ) + posWP;
	posLight = mul( mrotWP, posLight - posWP ) + posWP;


	// WY rotation
	MATRIX3 mrotWY = rotm( axisWY, joint_angle[WRIST_YAW] * RAD );

	posWR = mul( mrotWY, posWR - posWY ) + posWY;
	axisWR = mul( mrotWY, axisWR );

	posEE = mul( mrotWY, posEE - posWY ) + posWY;
	rotEE = mul( mrotWY, rotEE );
	posCCTVEE = mul( mrotWY, posCCTVEE - posWY ) + posWY;
	posLight = mul( mrotWY, posLight - posWY ) + posWY;


	// WR rotation
	// EE
	// CCTV
	// light
	MATRIX3 mrotWR = rotm( axisWR, joint_angle[WRIST_ROLL] * RAD );

	//posEE = mul( mrotWR, posEE - posWR ) + posWR;// no effect
	//dirEE = mul( mrotWR, dirEE );// no effect
	rotEE = mul( mrotWR, rotEE );
	{
		MATRIX3 mrotIK = rotm( axisWR, RMS_ROLLOUT_ANGLE * RAD );
		rotEEik = -mul( mrotIK, rotEE );
	}
	posCCTVEE = mul( mrotWR, posCCTVEE - posWR ) + posWR;
	posLight = mul( mrotWR, posLight - posWR ) + posWR;

	dirEE = -axisWR;

	posArmCoM = massprops::ArmCoM( posSP, posEP, posWP, posEE );
	return;
}

void RMS::GetMassPoints( massprops::Point& arm, massprops::Point& armStowed, massprops::Point& payload ) const
{
	arm.m = massprops::ARM_MASS;
	arm.r = posArmCoM + RMS_MESH_OFFSET;
	armStowed.m = -massprops::ARM_MASS;
	armStowed.r = RMS_ARM_COM_STOWED + RMS_MESH_OFFSET;
	payload.m = 0.0;
	payload.r = _V( 0.0, 0.0, 0.0 );
	if ((attachedPayload != NULL) && hAttach)
	{
		VECTOR3 cog;// in the local frame of the orbiter (origin in the c.g.)
		payload.m = GetMassAndCoGOfAttachedObject( STS(), hAttach, cog );
		payload.r = cog - STS()->GetOrbiterCoGOffset();// to the frame of the mesh
	}
}
