/******* SSV File Modification Notice *******
Date         Developer
2020/03/20   GLS
2020/04/07   GLS
2020/05/08   GLS
2020/05/17   GLS
2020/05/23   GLS
2020/06/20   GLS
2020/08/24   GLS
2021/08/23   GLS
2021/08/24   GLS
2021/10/23   GLS
2021/12/30   GLS
2022/01/10   GLS
2022/01/26   GLS
2022/02/17   GLS
2022/03/24   GLS
2022/03/26   GLS
2022/05/02   GLS
2022/06/10   GLS
2022/06/23   GLS
2022/08/05   GLS
2022/09/29   GLS
2022/10/28   GLS
2022/12/17   GLS
2022/12/21   indy91
2022/12/23   GLS
2022/12/28   GLS
2022/12/31   GLS
2026/04/05   indy91
2026/10/14   google0101-ryan
********************************************/
#include "OrbitDAP.h"
#include "../../IDP.h"
#include "../../../vc/MDU.h"
#include <MathSSV.h>
#include "../../../ParameterValues.h"
#include "RHC_SOP.h"
#include "THC_SOP.h"
#include "StateVectorSoftware.h"
#include "UniversalPointing.h"
#include "GNCUtilities.h"
#include "../../../RCSMath.h"
#include "../../../SSVOptions.h"
#include "../../../Atlantis.h"

namespace dps
{
	//Post Contact Thrusting
	constexpr double PCT_STAGE1 = 0.56;
	//period (s) for initial jet firing
	constexpr double PCT_STAGE2 = 1.52;
	//period (s) between jet firings
	constexpr double PCT_STAGE3 = 2.40;
	//period (s) for second jet firing

	constexpr int NoseRotThrusters[6] = {0, 3, 6, 8, 10, 13};
	constexpr int NosePitchThrusters[2] = {0, 3};
	constexpr int NoseYawThrusters[2] = {6, 8};
	//constexpr int NoseRollThrusters[2] = {10, 13};
	constexpr int AftRotThrusters[10] = {1, 2, 4, 5, 7, 9, 11, 12, 14, 15};
	constexpr int AftPitchThrusters[4] = {1, 2, 4, 5};
	constexpr int AftYawThrusters[2] = {7, 9};
	//constexpr int AftRollThrusters[4] = {11, 12, 14, 15};
	//RCS Thruster Groups

static void SaveAttManeuver(FILEHANDLE scn, char* item, const AttManeuver& maneuver)
{
	char cbuf[255];
	VECTOR3 eulerAngles = GetYZX_PYRAnglesFromMatrix(maneuver.tgtMatrix);
	sprintf_s(cbuf, 255, "%d %f %f %f", maneuver.Type, eulerAngles.data[PITCH], eulerAngles.data[YAW], eulerAngles.data[ROLL]);
	oapiWriteScenario_string(scn, item, cbuf);
}

static void LoadAttManeuver(const char* value, AttManeuver& maneuver)
{
	int nTemp;
	VECTOR3 vTemp;
	sscanf_s(value, "%d%lf%lf%lf", &nTemp, &vTemp.data[PITCH], &vTemp.data[YAW], &vTemp.data[ROLL]);

	if(nTemp == AttManeuver::MNVR || nTemp == AttManeuver::LVLH_TRK || nTemp == AttManeuver::UP) {
		maneuver.IsValid = true;
		maneuver.Type = static_cast<AttManeuver::TYPE>(nTemp);
		maneuver.tgtMatrix = GetRotationMatrixYZX(_V(vTemp.data[ROLL], vTemp.data[PITCH], vTemp.data[YAW]));
	}
}

OrbitDAP::OrbitDAP(SimpleGPCSystem* pGPC)
: SimpleGPCSoftware(pGPC, "OrbitDAP"),
OMSTVCControlP(5.0, 0.0, 0.5, -6.0, 6.0 ), OMSTVCControlY( 4.0, 0.0, 0.75, -7.0, 7.0 ), OMSTVCControlR(5.0, 0.0, 0.5, -6.0, 6.0 ),
ControlMode(RCS),
DAPMode(PRI), DAPSelect(A), DAPControlMode(INRTL), editDAP(-1),
ManeuverStatus(MNVR_OFF),
bFirstStep(true), lastStepdt(1.0),
PCTArmed(false), PCTActive(false),
pStateVector(NULL), pUniversalPointing(NULL)
{
	OMSTrim = _V(0.0, 0.0, 0.0);
	degReqdRates = _V(0.0, 0.0, 0.0);
	TransPulseDV = _V(0.0, 0.0, 0.0);
	ATT_ERR = _V(0.0, 0.0, 0.0);
	RATE_EST = _V(0, 0, 0);

	ActiveManeuver.IsValid = false;
	CurManeuver.IsValid = false;

	Torque.data[PITCH]=ORBITER_PITCH_TORQUE;
	Torque.data[YAW]=ORBITER_YAW_TORQUE;
	Torque.data[ROLL]=ORBITER_ROLL_TORQUE;

	// The below is a nominal iloaded ISS rendezvous DAP setup

	// DAP A (numbers for typical ISS DAP A1 - NOM)
	RotMode[0] = DISC_RATE;
	TransMode[0] = NORM;
	RotPulseInProg[0] = false;
	TransPulseInProg[0] = false;
	RotatingAxis[0] = false;
	NullingRates[0] = false;
	DAPConfiguration[0][0].PRI_ROT_RATE = 0.2;
	DAPConfiguration[0][0].PRI_ATT_DB = 5.0;
	DAPConfiguration[0][0].PRI_RATE_DB = 0.2;
	DAPConfiguration[0][0].PRI_ROT_PLS = 0.1;
	DAPConfiguration[0][0].PRI_COMP = 0.0;
	DAPConfiguration[0][0].PRI_P_OPTION = 0;
	DAPConfiguration[0][0].PRI_Y_OPTION = 0;
	DAPConfiguration[0][0].PRI_TRAN_PLS = 0.1;
	DAPConfiguration[0][0].ALT_RATE_DB = 0.2;
	DAPConfiguration[0][0].ALT_ON_TIME = 0.08;
	DAPConfiguration[0][0].ALT_DELAY = 0.0;
	DAPConfiguration[0][0].ALT_JET_OPT = 0;
	DAPConfiguration[0][0].ALT_JETS = 2;
	DAPConfiguration[0][0].VERN_ROT_RATE = 0.2;
	DAPConfiguration[0][0].VERN_ATT_DB = 1.0;
	DAPConfiguration[0][0].VERN_RATE_DB = 0.020;
	DAPConfiguration[0][0].VERN_ROT_PLS = 0.01;
	DAPConfiguration[0][0].VERN_COMP = 0.0;
	DAPConfiguration[0][0].VERN_CNTL_ACC = 0;

	// DAP A2 - PTC
	DAPConfiguration[0][1].PRI_ROT_RATE = 0.4;
	DAPConfiguration[0][1].PRI_ATT_DB = 1.0;
	DAPConfiguration[0][1].PRI_RATE_DB = 0.2;
	DAPConfiguration[0][1].PRI_ROT_PLS = 0.1;
	DAPConfiguration[0][1].PRI_COMP = 0;
	DAPConfiguration[0][1].PRI_P_OPTION = 0;
	DAPConfiguration[0][1].PRI_Y_OPTION = 0;
	DAPConfiguration[0][1].PRI_TRAN_PLS = 0.1;
	DAPConfiguration[0][1].ALT_RATE_DB = 0.2;
	DAPConfiguration[0][1].ALT_JET_OPT = 0;
	DAPConfiguration[0][1].ALT_JETS = 2;
	DAPConfiguration[0][1].ALT_ON_TIME = 0.08;
	DAPConfiguration[0][1].ALT_DELAY = 0;
	DAPConfiguration[0][1].VERN_ROT_RATE = 0.4;
	DAPConfiguration[0][1].VERN_ATT_DB = 1.0;
	DAPConfiguration[0][1].VERN_RATE_DB = 0.02;
	DAPConfiguration[0][1].VERN_ROT_PLS = 0.01;
	DAPConfiguration[0][1].VERN_COMP = 0;
	DAPConfiguration[0][1].VERN_CNTL_ACC = 0;

	// DAP A3 - Gravity Gradient
	DAPConfiguration[0][2].PRI_ROT_RATE = 0.2;
	DAPConfiguration[0][2].PRI_ATT_DB = 0.3;
	DAPConfiguration[0][2].PRI_RATE_DB = 0.2;
	DAPConfiguration[0][2].PRI_ROT_PLS = 0.1;
	DAPConfiguration[0][2].PRI_COMP = 0;
	DAPConfiguration[0][2].PRI_P_OPTION = 0;
	DAPConfiguration[0][2].PRI_Y_OPTION = 1;
	DAPConfiguration[0][2].PRI_TRAN_PLS = 0.1;
	DAPConfiguration[0][2].ALT_RATE_DB = 0.07;
	DAPConfiguration[0][2].ALT_JET_OPT = 0;
	DAPConfiguration[0][2].ALT_JETS = 1;
	DAPConfiguration[0][2].ALT_ON_TIME = 0.08;
	DAPConfiguration[0][2].ALT_DELAY = 0;
	DAPConfiguration[0][2].VERN_ROT_RATE = 0.008;
	DAPConfiguration[0][2].VERN_ATT_DB = 0.07;
	DAPConfiguration[0][2].VERN_RATE_DB = 0.01;
	DAPConfiguration[0][2].VERN_ROT_PLS = 0.01;
	DAPConfiguration[0][2].VERN_COMP = 0;
	DAPConfiguration[0][2].VERN_CNTL_ACC = 0;

	// DAP A4
	DAPConfiguration[0][3].PRI_ROT_RATE = 0.5;
	DAPConfiguration[0][3].PRI_ATT_DB = 3;
	DAPConfiguration[0][3].PRI_RATE_DB = 0.2;
	DAPConfiguration[0][3].PRI_ROT_PLS = 0.1;
	DAPConfiguration[0][3].PRI_COMP = 0;
	DAPConfiguration[0][3].PRI_P_OPTION = 0;
	DAPConfiguration[0][3].PRI_Y_OPTION = 0;
	DAPConfiguration[0][3].PRI_TRAN_PLS = 0.1;
	DAPConfiguration[0][3].ALT_RATE_DB = 0.2;
	DAPConfiguration[0][3].ALT_JET_OPT = 0;
	DAPConfiguration[0][3].ALT_JETS = 2;
	DAPConfiguration[0][3].ALT_ON_TIME = 0.08;
	DAPConfiguration[0][3].ALT_DELAY = 0;
	DAPConfiguration[0][3].VERN_ROT_RATE = 0.2;
	DAPConfiguration[0][3].VERN_ATT_DB = 1;
	DAPConfiguration[0][3].VERN_RATE_DB = 0.02;
	DAPConfiguration[0][3].VERN_ROT_PLS = 0.01;
	DAPConfiguration[0][3].VERN_COMP = 0;
	DAPConfiguration[0][3].VERN_CNTL_ACC = 0;

	// DAP A5 (loss of vern ALL)
	DAPConfiguration[0][4].PRI_ROT_RATE = 0.2;
	DAPConfiguration[0][4].PRI_ATT_DB = 5;
	DAPConfiguration[0][4].PRI_RATE_DB = 0.2;
	DAPConfiguration[0][4].PRI_ROT_PLS = 0.1;
	DAPConfiguration[0][4].PRI_COMP = 0;
	DAPConfiguration[0][4].PRI_P_OPTION = 0;
	DAPConfiguration[0][4].PRI_Y_OPTION = 0;
	DAPConfiguration[0][4].PRI_TRAN_PLS = 0.1;
	DAPConfiguration[0][4].ALT_RATE_DB = 0.07;
	DAPConfiguration[0][4].ALT_JET_OPT = 0;
	DAPConfiguration[0][4].ALT_JETS = 1;
	DAPConfiguration[0][4].ALT_ON_TIME = 0.08;
	DAPConfiguration[0][4].ALT_DELAY = 0;
	DAPConfiguration[0][4].VERN_ROT_RATE = 0.05;
	DAPConfiguration[0][4].VERN_ATT_DB = 1;
	DAPConfiguration[0][4].VERN_RATE_DB = 0.02;
	DAPConfiguration[0][4].VERN_ROT_PLS = 0.01;
	DAPConfiguration[0][4].VERN_COMP = 0;
	DAPConfiguration[0][4].VERN_CNTL_ACC = 0;

	// DAP A6 (loss of vern TAIL)
	DAPConfiguration[0][5].PRI_ROT_RATE = 0.2;
	DAPConfiguration[0][5].PRI_ATT_DB = 5;
	DAPConfiguration[0][5].PRI_RATE_DB = 0.2;
	DAPConfiguration[0][5].PRI_ROT_PLS = 0.1;
	DAPConfiguration[0][5].PRI_COMP = 0;
	DAPConfiguration[0][5].PRI_P_OPTION = 1;
	DAPConfiguration[0][5].PRI_Y_OPTION = 1;
	DAPConfiguration[0][5].PRI_TRAN_PLS = 0.1;
	DAPConfiguration[0][5].ALT_RATE_DB = 0.07;
	DAPConfiguration[0][5].ALT_JET_OPT = 1;
	DAPConfiguration[0][5].ALT_JETS = 1;
	DAPConfiguration[0][5].ALT_ON_TIME = 0.08;
	DAPConfiguration[0][5].ALT_DELAY = 0;
	DAPConfiguration[0][5].VERN_ROT_RATE = 0.05;
	DAPConfiguration[0][5].VERN_ATT_DB = 1;
	DAPConfiguration[0][5].VERN_RATE_DB = 0.02;
	DAPConfiguration[0][5].VERN_ROT_PLS = 0.01;
	DAPConfiguration[0][5].VERN_COMP = 0;
	DAPConfiguration[0][5].VERN_CNTL_ACC = 0;

	// DAP A7 (rendezvous)
	DAPConfiguration[0][6].PRI_ROT_RATE = 0.2;
	DAPConfiguration[0][6].PRI_ATT_DB = 2;
	DAPConfiguration[0][6].PRI_RATE_DB = 0.2;
	DAPConfiguration[0][6].PRI_ROT_PLS = 0.1;
	DAPConfiguration[0][6].PRI_COMP = 0;
	DAPConfiguration[0][6].PRI_P_OPTION = 0;
	DAPConfiguration[0][6].PRI_Y_OPTION = 0;
	DAPConfiguration[0][6].PRI_TRAN_PLS = 0.1;
	DAPConfiguration[0][6].ALT_RATE_DB = 0.1;
	DAPConfiguration[0][6].ALT_JET_OPT = 0;
	DAPConfiguration[0][6].ALT_JETS = 2;
	DAPConfiguration[0][6].ALT_ON_TIME = 0.08;
	DAPConfiguration[0][6].ALT_DELAY = 0;
	DAPConfiguration[0][6].VERN_ROT_RATE = 0.0160;
	DAPConfiguration[0][6].VERN_ATT_DB = 1;
	DAPConfiguration[0][6].VERN_RATE_DB = 0.02;
	DAPConfiguration[0][6].VERN_ROT_PLS = 0.01;
	DAPConfiguration[0][6].VERN_COMP = 0;
	DAPConfiguration[0][6].VERN_CNTL_ACC = 0;

	// DAP A8 (terminal phase)
	DAPConfiguration[0][7].PRI_ROT_RATE = 0.2;
	DAPConfiguration[0][7].PRI_ATT_DB = 2;
	DAPConfiguration[0][7].PRI_RATE_DB = 0.2;
	DAPConfiguration[0][7].PRI_ROT_PLS = 0.1;
	DAPConfiguration[0][7].PRI_COMP = 0;
	DAPConfiguration[0][7].PRI_P_OPTION = 0;
	DAPConfiguration[0][7].PRI_Y_OPTION = 0;
	DAPConfiguration[0][7].PRI_TRAN_PLS = 0.1;
	DAPConfiguration[0][7].ALT_RATE_DB = 0.1;
	DAPConfiguration[0][7].ALT_JET_OPT = 0;
	DAPConfiguration[0][7].ALT_JETS = 2;
	DAPConfiguration[0][7].ALT_ON_TIME = 0.08;
	DAPConfiguration[0][7].ALT_DELAY = 0;
	DAPConfiguration[0][7].VERN_ROT_RATE = 0.05;
	DAPConfiguration[0][7].VERN_ATT_DB = 1;
	DAPConfiguration[0][7].VERN_RATE_DB = 0.02;
	DAPConfiguration[0][7].VERN_ROT_PLS = 0.05;
	DAPConfiguration[0][7].VERN_COMP = 0;
	DAPConfiguration[0][7].VERN_CNTL_ACC = 0;

	// DAP A9 (Prox ops / flyaround)
	DAPConfiguration[0][8].PRI_ROT_RATE = 0.13;
	DAPConfiguration[0][8].PRI_ATT_DB = 2;
	DAPConfiguration[0][8].PRI_RATE_DB = 0.1;
	DAPConfiguration[0][8].PRI_ROT_PLS = 0.1;
	DAPConfiguration[0][8].PRI_COMP = 0;
	DAPConfiguration[0][8].PRI_P_OPTION = 1;
	DAPConfiguration[0][8].PRI_Y_OPTION = 1;
	DAPConfiguration[0][8].PRI_TRAN_PLS = 0.05;
	DAPConfiguration[0][8].ALT_RATE_DB = 0.1;
	DAPConfiguration[0][8].ALT_JET_OPT = 1;
	DAPConfiguration[0][8].ALT_JETS = 2;
	DAPConfiguration[0][8].ALT_ON_TIME = 0.08;
	DAPConfiguration[0][8].ALT_DELAY = 0;
	DAPConfiguration[0][8].VERN_ROT_RATE = 0.13;
	DAPConfiguration[0][8].VERN_ATT_DB = 1;
	DAPConfiguration[0][8].VERN_RATE_DB = 0.02;
	DAPConfiguration[0][8].VERN_ROT_PLS = 0.05;
	DAPConfiguration[0][8].VERN_COMP = 0;
	DAPConfiguration[0][8].VERN_CNTL_ACC = 0;

	// DAP A10 (docking)
	DAPConfiguration[0][9].PRI_ROT_RATE = 0.05;
	DAPConfiguration[0][9].PRI_ATT_DB = 0.6;
	DAPConfiguration[0][9].PRI_RATE_DB = 0.1;
	DAPConfiguration[0][9].PRI_ROT_PLS = 0.1;
	DAPConfiguration[0][9].PRI_COMP = 0;
	DAPConfiguration[0][9].PRI_P_OPTION = 1;
	DAPConfiguration[0][9].PRI_Y_OPTION = 1;
	DAPConfiguration[0][9].PRI_TRAN_PLS = 0.05;
	DAPConfiguration[0][9].ALT_RATE_DB = 0.1;
	DAPConfiguration[0][9].ALT_JET_OPT = 1;
	DAPConfiguration[0][9].ALT_JETS = 2;
	DAPConfiguration[0][9].ALT_ON_TIME = 0.08;
	DAPConfiguration[0][9].ALT_DELAY = 0;
	DAPConfiguration[0][9].VERN_ROT_RATE = 0.05;
	DAPConfiguration[0][9].VERN_ATT_DB = 0.5;
	DAPConfiguration[0][9].VERN_RATE_DB = 0.02;
	DAPConfiguration[0][9].VERN_ROT_PLS = 0.05;
	DAPConfiguration[0][9].VERN_COMP = 0;
	DAPConfiguration[0][9].VERN_CNTL_ACC = 0;

	// DAP A11 (auto reboost)
	DAPConfiguration[0][10].PRI_ROT_RATE = 0.05;
	DAPConfiguration[0][10].PRI_ATT_DB = 10;
	DAPConfiguration[0][10].PRI_RATE_DB = 0.2;
	DAPConfiguration[0][10].PRI_ROT_PLS = 0.1;
	DAPConfiguration[0][10].PRI_COMP = 0;
	DAPConfiguration[0][10].PRI_P_OPTION = 1;
	DAPConfiguration[0][10].PRI_Y_OPTION = 1;
	DAPConfiguration[0][10].PRI_TRAN_PLS = 0.05;
	DAPConfiguration[0][10].ALT_RATE_DB = 0.035;
	DAPConfiguration[0][10].ALT_JET_OPT = 1;
	DAPConfiguration[0][10].ALT_JETS = 1;
	DAPConfiguration[0][10].ALT_ON_TIME = 0.08;
	DAPConfiguration[0][10].ALT_DELAY = 6;
	DAPConfiguration[0][10].VERN_ROT_RATE = 0.1;
	DAPConfiguration[0][10].VERN_ATT_DB = 5.0;
	DAPConfiguration[0][10].VERN_RATE_DB = 0.05;
	DAPConfiguration[0][10].VERN_ROT_PLS = 0.01;
	DAPConfiguration[0][10].VERN_COMP = 0;
	DAPConfiguration[0][10].VERN_CNTL_ACC = 2;

	// DAP A12 (Mated hold (VRCS and ALT)
	DAPConfiguration[0][11].PRI_ROT_RATE = 0.05;
	DAPConfiguration[0][11].PRI_ATT_DB = 1;
	DAPConfiguration[0][11].PRI_RATE_DB = 0.2;
	DAPConfiguration[0][11].PRI_ROT_PLS = 0.04;
	DAPConfiguration[0][11].PRI_COMP = 0;
	DAPConfiguration[0][11].PRI_P_OPTION = 1;
	DAPConfiguration[0][11].PRI_Y_OPTION = 1;
	DAPConfiguration[0][11].PRI_TRAN_PLS = 0.01;
	DAPConfiguration[0][11].ALT_RATE_DB = 0.035;
	DAPConfiguration[0][11].ALT_JET_OPT = 1;
	DAPConfiguration[0][11].ALT_JETS = 1;
	DAPConfiguration[0][11].ALT_ON_TIME = 0.08;
	DAPConfiguration[0][11].ALT_DELAY = 6;
	DAPConfiguration[0][11].VERN_ROT_RATE = 0.01;
	DAPConfiguration[0][11].VERN_ATT_DB = 1;
	DAPConfiguration[0][11].VERN_RATE_DB = 0.02;
	DAPConfiguration[0][11].VERN_ROT_PLS = 0.002;
	DAPConfiguration[0][11].VERN_COMP = 0;
	DAPConfiguration[0][11].VERN_CNTL_ACC = 2;

	// DAP A14 (Orbiter alone SRMS/OBSS)
	DAPConfiguration[0][13].PRI_ROT_RATE = 0.2;
	DAPConfiguration[0][13].PRI_ATT_DB = 5;
	DAPConfiguration[0][13].PRI_RATE_DB = 0.2;
	DAPConfiguration[0][13].PRI_ROT_PLS = 0.1;
	DAPConfiguration[0][13].PRI_COMP = 0;
	DAPConfiguration[0][13].PRI_P_OPTION = 1;
	DAPConfiguration[0][13].PRI_Y_OPTION = 1;
	DAPConfiguration[0][13].PRI_TRAN_PLS = 0.1;
	DAPConfiguration[0][13].ALT_RATE_DB = 0.2;
	DAPConfiguration[0][13].ALT_JET_OPT = 1;
	DAPConfiguration[0][13].ALT_JETS = 1;
	DAPConfiguration[0][13].ALT_ON_TIME = 0.08;
	DAPConfiguration[0][13].ALT_DELAY = 6;
	DAPConfiguration[0][13].VERN_ROT_RATE = 0.2;
	DAPConfiguration[0][13].VERN_ATT_DB = 3;
	DAPConfiguration[0][13].VERN_RATE_DB = 0.2;
	DAPConfiguration[0][13].VERN_ROT_PLS = 0.01;
	DAPConfiguration[0][13].VERN_COMP = 0;
	DAPConfiguration[0][13].VERN_CNTL_ACC = 1;

	RotMode[1] = DISC_RATE;
	TransMode[1] = NORM;
	RotPulseInProg[1] = false;
	TransPulseInProg[1] = false;
	RotatingAxis[1] = false;
	NullingRates[1] = false;

	// DAP B1 (OMS & RCS burns)
	DAPConfiguration[1][0].PRI_ROT_RATE = 0.5;
	DAPConfiguration[1][0].PRI_ATT_DB = 3;
	DAPConfiguration[1][0].PRI_RATE_DB = 0.2;
	DAPConfiguration[1][0].PRI_ROT_PLS = 0.04;
	DAPConfiguration[1][0].PRI_COMP = 0.0;
	DAPConfiguration[1][0].PRI_P_OPTION = 0;
	DAPConfiguration[1][0].PRI_Y_OPTION = 0;
	DAPConfiguration[1][0].PRI_TRAN_PLS = 0.1;
	DAPConfiguration[1][0].ALT_RATE_DB = 0.2;
	DAPConfiguration[1][0].ALT_JET_OPT = 0;
	DAPConfiguration[1][0].ALT_JETS = 2;
	DAPConfiguration[1][0].ALT_ON_TIME = 0.08;
	DAPConfiguration[1][0].ALT_DELAY = 0.0;
	DAPConfiguration[1][0].VERN_ROT_RATE = 0.2;
	DAPConfiguration[1][0].VERN_ATT_DB = 1;
	DAPConfiguration[1][0].VERN_RATE_DB = 0.020;
	DAPConfiguration[1][0].VERN_ROT_PLS = 0.002;
	DAPConfiguration[1][0].VERN_COMP = 0.0;
	DAPConfiguration[1][0].VERN_CNTL_ACC = 0;

	// DAP B2 (Loss of vern tail-only)
	DAPConfiguration[1][1].PRI_ROT_RATE = 0.2;
	DAPConfiguration[1][1].PRI_ATT_DB = 3;
	DAPConfiguration[1][1].PRI_RATE_DB = 0.2;
	DAPConfiguration[1][1].PRI_ROT_PLS = 0.04;
	DAPConfiguration[1][1].PRI_COMP = 0.0;
	DAPConfiguration[1][1].PRI_P_OPTION = 1;
	DAPConfiguration[1][1].PRI_Y_OPTION = 1;
	DAPConfiguration[1][1].PRI_TRAN_PLS = 0.02;
	DAPConfiguration[1][1].ALT_RATE_DB = 0.07;
	DAPConfiguration[1][1].ALT_JET_OPT = 1;
	DAPConfiguration[1][1].ALT_JETS = 1;
	DAPConfiguration[1][1].ALT_ON_TIME = 0.08;
	DAPConfiguration[1][1].ALT_DELAY = 0.0;
	DAPConfiguration[1][1].VERN_ROT_RATE = 0.2;
	DAPConfiguration[1][1].VERN_ATT_DB = 1;
	DAPConfiguration[1][1].VERN_RATE_DB = 0.020;
	DAPConfiguration[1][1].VERN_ROT_PLS = 0.002;
	DAPConfiguration[1][1].VERN_COMP = 0.0;
	DAPConfiguration[1][1].VERN_CNTL_ACC = 0;

	// DAP B3 (Loss of vern all)
	DAPConfiguration[1][2].PRI_ROT_RATE = 0.2;
	DAPConfiguration[1][2].PRI_ATT_DB = 3;
	DAPConfiguration[1][2].PRI_RATE_DB = 0.2;
	DAPConfiguration[1][2].PRI_ROT_PLS = 0.04;
	DAPConfiguration[1][2].PRI_COMP = 0.0;
	DAPConfiguration[1][2].PRI_P_OPTION = 0;
	DAPConfiguration[1][2].PRI_Y_OPTION = 0;
	DAPConfiguration[1][2].PRI_TRAN_PLS = 0.02;
	DAPConfiguration[1][2].ALT_RATE_DB = 0.07;
	DAPConfiguration[1][2].ALT_JET_OPT = 0;
	DAPConfiguration[1][2].ALT_JETS = 1;
	DAPConfiguration[1][2].ALT_ON_TIME = 0.08;
	DAPConfiguration[1][2].ALT_DELAY = 0.0;
	DAPConfiguration[1][2].VERN_ROT_RATE = 0.2;
	DAPConfiguration[1][2].VERN_ATT_DB = 1;
	DAPConfiguration[1][2].VERN_RATE_DB = 0.020;
	DAPConfiguration[1][2].VERN_ROT_PLS = 0.002;
	DAPConfiguration[1][2].VERN_COMP = 0.0;
	DAPConfiguration[1][2].VERN_CNTL_ACC = 0;

	// DAP B4
	DAPConfiguration[1][3].PRI_ROT_RATE = 0.2;
	DAPConfiguration[1][3].PRI_ATT_DB = 0.3;
	DAPConfiguration[1][3].PRI_RATE_DB = 0.2;
	DAPConfiguration[1][3].PRI_ROT_PLS = 0.04;
	DAPConfiguration[1][3].PRI_COMP = 0.0;
	DAPConfiguration[1][3].PRI_P_OPTION = 1;
	DAPConfiguration[1][3].PRI_Y_OPTION = 1;
	DAPConfiguration[1][3].PRI_TRAN_PLS = 0.02;
	DAPConfiguration[1][3].ALT_RATE_DB = 0.1;
	DAPConfiguration[1][3].ALT_JET_OPT = 0;
	DAPConfiguration[1][3].ALT_JETS = 2;
	DAPConfiguration[1][3].ALT_ON_TIME = 0.08;
	DAPConfiguration[1][3].ALT_DELAY = 0.0;
	DAPConfiguration[1][3].VERN_ROT_RATE = 0.008;
	DAPConfiguration[1][3].VERN_ATT_DB = 0.1;
	DAPConfiguration[1][3].VERN_RATE_DB = 0.010;
	DAPConfiguration[1][3].VERN_ROT_PLS = 0.002;
	DAPConfiguration[1][3].VERN_COMP = 0.0;
	DAPConfiguration[1][3].VERN_CNTL_ACC = 0;

	// DAP B5 (COAS/HUD cal)
	DAPConfiguration[1][4].PRI_ROT_RATE = 0.2;
	DAPConfiguration[1][4].PRI_ATT_DB = 3;
	DAPConfiguration[1][4].PRI_RATE_DB = 0.2;
	DAPConfiguration[1][4].PRI_ROT_PLS = 0.04;
	DAPConfiguration[1][4].PRI_COMP = 0.0;
	DAPConfiguration[1][4].PRI_P_OPTION = 1;
	DAPConfiguration[1][4].PRI_Y_OPTION = 1;
	DAPConfiguration[1][4].PRI_TRAN_PLS = 0.02;
	DAPConfiguration[1][4].ALT_RATE_DB = 0.2;
	DAPConfiguration[1][4].ALT_JET_OPT = 0;
	DAPConfiguration[1][4].ALT_JETS = 2;
	DAPConfiguration[1][4].ALT_ON_TIME = 0.08;
	DAPConfiguration[1][4].ALT_DELAY = 0.0;
	DAPConfiguration[1][4].VERN_ROT_RATE = 0.0160;
	DAPConfiguration[1][4].VERN_ATT_DB = 0.033;
	DAPConfiguration[1][4].VERN_RATE_DB = 0.020;
	DAPConfiguration[1][4].VERN_ROT_PLS = 0.002;
	DAPConfiguration[1][4].VERN_COMP = 0.0;
	DAPConfiguration[1][4].VERN_CNTL_ACC = 0;

	// DAP B6
	DAPConfiguration[1][5].PRI_ROT_RATE = 0.5;
	DAPConfiguration[1][5].PRI_ATT_DB = 3;
	DAPConfiguration[1][5].PRI_RATE_DB = 0.2;
	DAPConfiguration[1][5].PRI_ROT_PLS = 0.04;
	DAPConfiguration[1][5].PRI_COMP = 0.0;
	DAPConfiguration[1][5].PRI_P_OPTION = 0;
	DAPConfiguration[1][5].PRI_Y_OPTION = 0;
	DAPConfiguration[1][5].PRI_TRAN_PLS = 0.02;
	DAPConfiguration[1][5].ALT_RATE_DB = 0.2;
	DAPConfiguration[1][5].ALT_JET_OPT = 0;
	DAPConfiguration[1][5].ALT_JETS = 2;
	DAPConfiguration[1][5].ALT_ON_TIME = 0.08;
	DAPConfiguration[1][5].ALT_DELAY = 0.0;
	DAPConfiguration[1][5].VERN_ROT_RATE = 0.2;
	DAPConfiguration[1][5].VERN_ATT_DB = 1;
	DAPConfiguration[1][5].VERN_RATE_DB = 0.020;
	DAPConfiguration[1][5].VERN_ROT_PLS = 0.002;
	DAPConfiguration[1][5].VERN_COMP = 0.0;
	DAPConfiguration[1][5].VERN_CNTL_ACC = 0;

	// DAP B7 (rendezvous)
	DAPConfiguration[1][6].PRI_ROT_RATE = 0.5;
	DAPConfiguration[1][6].PRI_ATT_DB = 2;
	DAPConfiguration[1][6].PRI_RATE_DB = 0.2;
	DAPConfiguration[1][6].PRI_ROT_PLS = 0.04;
	DAPConfiguration[1][6].PRI_COMP = 0.0;
	DAPConfiguration[1][6].PRI_P_OPTION = 0;
	DAPConfiguration[1][6].PRI_Y_OPTION = 0;
	DAPConfiguration[1][6].PRI_TRAN_PLS = 0.05;
	DAPConfiguration[1][6].ALT_RATE_DB = 0.1;
	DAPConfiguration[1][6].ALT_JET_OPT = 0;
	DAPConfiguration[1][6].ALT_JETS = 2;
	DAPConfiguration[1][6].ALT_ON_TIME = 0.08;
	DAPConfiguration[1][6].ALT_DELAY = 0.0;
	DAPConfiguration[1][6].VERN_ROT_RATE = 0.2;
	DAPConfiguration[1][6].VERN_ATT_DB = 1;
	DAPConfiguration[1][6].VERN_RATE_DB = 0.020;
	DAPConfiguration[1][6].VERN_ROT_PLS = 0.002;
	DAPConfiguration[1][6].VERN_COMP = 0.0;
	DAPConfiguration[1][6].VERN_CNTL_ACC = 0;

	// DAP B8 (terminal phase)
	DAPConfiguration[1][7].PRI_ROT_RATE = 0.05;
	DAPConfiguration[1][7].PRI_ATT_DB = 2;
	DAPConfiguration[1][7].PRI_RATE_DB = 0.2;
	DAPConfiguration[1][7].PRI_ROT_PLS = 0.04;
	DAPConfiguration[1][7].PRI_COMP = 0.0;
	DAPConfiguration[1][7].PRI_P_OPTION = 0;
	DAPConfiguration[1][7].PRI_Y_OPTION = 0;
	DAPConfiguration[1][7].PRI_TRAN_PLS = 0.05;
	DAPConfiguration[1][7].ALT_RATE_DB = 0.1;
	DAPConfiguration[1][7].ALT_JET_OPT = 0;
	DAPConfiguration[1][7].ALT_JETS = 2;
	DAPConfiguration[1][7].ALT_ON_TIME = 0.08;
	DAPConfiguration[1][7].ALT_DELAY = 0.0;
	DAPConfiguration[1][7].VERN_ROT_RATE = 0.05;
	DAPConfiguration[1][7].VERN_ATT_DB = 1;
	DAPConfiguration[1][7].VERN_RATE_DB = 0.020;
	DAPConfiguration[1][7].VERN_ROT_PLS = 0.020;
	DAPConfiguration[1][7].VERN_COMP = 0.0;
	DAPConfiguration[1][7].VERN_CNTL_ACC = 0;

	// DAP B9 (prox ops/flyaround)
	DAPConfiguration[1][8].PRI_ROT_RATE = 0.13;
	DAPConfiguration[1][8].PRI_ATT_DB = 2;
	DAPConfiguration[1][8].PRI_RATE_DB = 0.1;
	DAPConfiguration[1][8].PRI_ROT_PLS = 0.04;
	DAPConfiguration[1][8].PRI_COMP = 0.0;
	DAPConfiguration[1][8].PRI_P_OPTION = 1;
	DAPConfiguration[1][8].PRI_Y_OPTION = 1;
	DAPConfiguration[1][8].PRI_TRAN_PLS = 0.01;
	DAPConfiguration[1][8].ALT_RATE_DB = 0.1;
	DAPConfiguration[1][8].ALT_JET_OPT = 1;
	DAPConfiguration[1][8].ALT_JETS = 2;
	DAPConfiguration[1][8].ALT_ON_TIME = 0.08;
	DAPConfiguration[1][8].ALT_DELAY = 10.0;
	DAPConfiguration[1][8].VERN_ROT_RATE = 0.13;
	DAPConfiguration[1][8].VERN_ATT_DB = 1;
	DAPConfiguration[1][8].VERN_RATE_DB = 0.020;
	DAPConfiguration[1][8].VERN_ROT_PLS = 0.020;
	DAPConfiguration[1][8].VERN_COMP = 0.0;
	DAPConfiguration[1][8].VERN_CNTL_ACC = 0;

	// DAP B10 (docking)
	DAPConfiguration[1][9].PRI_ROT_RATE = 0.05;
	DAPConfiguration[1][9].PRI_ATT_DB = 0.6;
	DAPConfiguration[1][9].PRI_RATE_DB = 0.1;
	DAPConfiguration[1][9].PRI_ROT_PLS = 0.04;
	DAPConfiguration[1][9].PRI_COMP = 0.0;
	DAPConfiguration[1][9].PRI_P_OPTION = 1;
	DAPConfiguration[1][9].PRI_Y_OPTION = 1;
	DAPConfiguration[1][9].PRI_TRAN_PLS = 0.01;
	DAPConfiguration[1][9].ALT_RATE_DB = 0.1;
	DAPConfiguration[1][9].ALT_JET_OPT = 1;
	DAPConfiguration[1][9].ALT_JETS = 2;
	DAPConfiguration[1][9].ALT_ON_TIME = 0.08;
	DAPConfiguration[1][9].ALT_DELAY = 0.0;
	DAPConfiguration[1][9].VERN_ROT_RATE = 0.05;
	DAPConfiguration[1][9].VERN_ATT_DB = 0.5;
	DAPConfiguration[1][9].VERN_RATE_DB = 0.020;
	DAPConfiguration[1][9].VERN_ROT_PLS = 0.020;
	DAPConfiguration[1][9].VERN_COMP = 0.0;
	DAPConfiguration[1][9].VERN_CNTL_ACC = 0;

	// DAP B12 (mated maneuvers VRCS & ALT)
	DAPConfiguration[1][11].PRI_ROT_RATE = 0.05;
	DAPConfiguration[1][11].PRI_ATT_DB = 5;
	DAPConfiguration[1][11].PRI_RATE_DB = 0.2;
	DAPConfiguration[1][11].PRI_ROT_PLS = 0.04;
	DAPConfiguration[1][11].PRI_COMP = 0.0;
	DAPConfiguration[1][11].PRI_P_OPTION = 1;
	DAPConfiguration[1][11].PRI_Y_OPTION = 1;
	DAPConfiguration[1][11].PRI_TRAN_PLS = 0.01;
	DAPConfiguration[1][11].ALT_RATE_DB = 0.035;
	DAPConfiguration[1][11].ALT_JET_OPT = 1;
	DAPConfiguration[1][11].ALT_JETS = 1;
	DAPConfiguration[1][11].ALT_ON_TIME = 0.08;
	DAPConfiguration[1][11].ALT_DELAY = 6;
	DAPConfiguration[1][11].VERN_ROT_RATE = 0.1;
	DAPConfiguration[1][11].VERN_ATT_DB = 3;
	DAPConfiguration[1][11].VERN_RATE_DB = 0.050;
	DAPConfiguration[1][11].VERN_ROT_PLS = 0.002;
	DAPConfiguration[1][11].VERN_COMP = 0.0;
	DAPConfiguration[1][11].VERN_CNTL_ACC = 2;

	// edit DAP
	RotMode[2] = DISC_RATE;
	TransMode[2] = NORM;
	RotPulseInProg[2] = false;
	TransPulseInProg[2] = false;
	RotatingAxis[2] = false;
	NullingRates[2] = false;
	DAPConfiguration[2][0].PRI_ROT_RATE = 2.0;
	DAPConfiguration[2][0].PRI_ATT_DB = 5.0;
	DAPConfiguration[2][0].PRI_RATE_DB = 0.2;
	DAPConfiguration[2][0].PRI_ROT_PLS = 0.1;
	DAPConfiguration[2][0].PRI_COMP = 0.0;
	DAPConfiguration[2][0].PRI_P_OPTION = 0;
	DAPConfiguration[2][0].PRI_Y_OPTION = 0;
	DAPConfiguration[2][0].PRI_TRAN_PLS = 0.1;
	DAPConfiguration[2][0].ALT_RATE_DB = 0.2;
	DAPConfiguration[2][0].ALT_ON_TIME = 0.08;
	DAPConfiguration[2][0].ALT_DELAY = 0.0;
	DAPConfiguration[2][0].ALT_JET_OPT = 0;
	DAPConfiguration[2][0].ALT_JETS = 2;
	DAPConfiguration[2][0].VERN_ROT_RATE = 0.1;
	DAPConfiguration[2][0].VERN_ATT_DB = 1.0;
	DAPConfiguration[2][0].VERN_RATE_DB = 0.020;
	DAPConfiguration[2][0].VERN_ROT_PLS = 0.01;
	DAPConfiguration[2][0].VERN_COMP = 0.0;
	DAPConfiguration[2][0].VERN_CNTL_ACC = 0;

	activeAProfile = 0;
	activeBProfile = 0;
}

OrbitDAP::~OrbitDAP()
{
}

void OrbitDAP::InitOMSTVC( const VECTOR3& Trim, CONTROL_MODE Mode )
{
	OMSTVCControlP.Reset();
	OMSTVCControlY.Reset();
	OMSTVCControlR.Reset();
	OMSTrim = Trim;
	OMSAttBias = _V( 0.0, 0.0, 0.0 );
	// initial gimbal command
	if (Mode != RIGHT_OMS)
	{
		WriteCOMPOOL_VS( SCP_OMSL_PITCH_YAW_CMD, 1, static_cast<float>(range( -5.0, OMSTrim.data[0], 5.0 )), 2 ); 
		WriteCOMPOOL_VS( SCP_OMSL_PITCH_YAW_CMD, 2, static_cast<float>(range( -8.0, OMSTrim.data[1], 8.0 )), 2 ); 
	}
	if (Mode != LEFT_OMS)
	{
		WriteCOMPOOL_VS( SCP_OMSR_PITCH_YAW_CMD, 1, static_cast<float>(range( -5.0, OMSTrim.data[0], 5.0 )), 2 ); 
		WriteCOMPOOL_VS( SCP_OMSR_PITCH_YAW_CMD, 2, static_cast<float>(range( -8.0, OMSTrim.data[2], 8.0 )), 2 ); 
	}
	return;
}

void OrbitDAP::UseOMSTVC( CONTROL_MODE Mode )
{
	if(Mode != RCS) ControlMode = Mode;
	return;
}

void OrbitDAP::UseRCS()
{
	ControlMode = RCS;
}

OrbitDAP::DAP_CONTROL_MODE OrbitDAP::GetDAPMode() const
{
	return DAPControlMode;
}

double OrbitDAP::GetDAPRate() const
{
	return degRotRate;
}

double OrbitDAP::GetDAPDeadband() const
{
	return degAttDeadband;
}

VECTOR3 OrbitDAP::Get_RATE_EST() const
{
	return RATE_EST;
}

bool OrbitDAP::Get_Preburn_Mnvr_In_Progress() const
{
	return false; // TBD
}

void OrbitDAP::ManeuverToINRTLAttitude(const VECTOR3& degINRTLAtt)
{
	//VECTOR3 radOrbiterAtt = ConvertAnglesBetweenM50AndOrbiter(degINRTLAtt*RAD, true);
	MATRIX3 OrbiterAtt = GetRotationMatrixYZX(_V(degINRTLAtt.data[ROLL], degINRTLAtt.data[PITCH], degINRTLAtt.data[YAW])*RAD);
	LoadCurINRTLManeuver(OrbiterAtt);
}

void OrbitDAP::ManeuverToUPAttitude()
{
	CurManeuver.IsValid = true;
	CurManeuver.tgtMatrix = _M(1, 0, 0, 0, 1, 0, 0, 0, 1); // Does not matter, Universal Pointing supplies the attitude directly
	CurManeuver.Type = AttManeuver::UP;
	if (DAPControlMode == AUTO) StartCurManeuver();
}

void OrbitDAP::CancelManeuver()
{
	// Old Universal Pointing item 21 (cancel) code
	CurManeuver.IsValid = false;
	RotatingAxis[YAW] = false;
	RotatingAxis[PITCH] = false;
	RotatingAxis[ROLL] = false;
	DAPControlMode = INRTL;
	StartManeuver(curM50Matrix, AttManeuver::MNVR);
}

void OrbitDAP::LoadCurINRTLManeuver(const MATRIX3& tgtMatrixM50)
{
	CurManeuver.IsValid = true;
	CurManeuver.tgtMatrix = tgtMatrixM50;
	CurManeuver.Type = AttManeuver::MNVR;
	if(DAPControlMode == AUTO) StartCurManeuver();
}

void OrbitDAP::StartCurManeuver()
{
	if(CurManeuver.IsValid) {
		//if(CurManeuver.Type == AttManeuver::MNVR) StartINRTLManeuver(CurManeuver.radTargetAttOrbiter);
		//else if(CurManeuver.Type == AttManeuver::TRK) StartLVLHManeuver(CurManeuver.radTargetLVLHAtt);
		StartManeuver(CurManeuver.tgtMatrix, CurManeuver.Type);
	}
}

void OrbitDAP::StartManeuver(const MATRIX3& tgtAtt, AttManeuver::TYPE type)
{
	ActiveManeuver.tgtMatrix = tgtAtt;
	ActiveManeuver.Type = type;
	ActiveManeuver.IsValid = true;

	ManeuverStatus = MNVR_IN_PROGRESS;

	if(ActiveManeuver.Type == AttManeuver::MNVR) {
		degNullRates = _V(0, 0, 0);

		// calculate time to reach target attitude
		VECTOR3 Axis;
		MATRIX3 AttError = GetRotationErrorMatrix(curM50Matrix, tgtAtt);
		double Angle = CalcEulerAngle(AttError, Axis);
		mnvrCompletionMET = STS()->GetMET() + (Angle*DEG)/degRotRate;
		lastUpdateTime = 0.0;
	}
	else if (ActiveManeuver.Type == AttManeuver::LVLH_TRK) {
		mnvrCompletionMET = STS()->GetMET();
		lastUpdateTime = -100.0;
	}
}

bool OrbitDAP::GetRHCRequiredRates()
{
	bool outOfDetent = false;
	double RHC[3];
	RHC[0] = pRHC_SOP->GetPitchCommand();
	RHC[1] = pRHC_SOP->GetYawCommand();
	RHC[2] = pRHC_SOP->GetRollCommand();
	bool RHCdetent[3];
	RHCdetent[0] = pRHC_SOP->GetPitchDetent();
	RHCdetent[1] = pRHC_SOP->GetYawDetent();
	RHCdetent[2] = pRHC_SOP->GetRollDetent();
	bool RHCpastsoftstop[3];
	RHCpastsoftstop[0] = pRHC_SOP->GetPitchPastSoftStop();
	RHCpastsoftstop[1] = pRHC_SOP->GetYawPastSoftStop();
	RHCpastsoftstop[2] = pRHC_SOP->GetRollPastSoftStop();

	for(unsigned int i=0;i<3;i++) {
		if (RHCdetent[i] == false)
		{
			outOfDetent = true;
			if (RHCpastsoftstop[i] == false)
			{
				if(RotMode[i]==DISC_RATE) { // DISC RATE
					// if RHC was pushed past soft stop previously (high rotation rates), maintain high rotation rate; otherwise rotate at specified rate
					degReqdRates.data[i] = max(degRotRate, abs(degAngularVelocity.data[i]))*sign( RHC[i] );
				}
				else if(!RotPulseInProg[i]) { // PULSE
					degReqdRates.data[i] = degReqdRates.data[i]+degRotPulse*sign( RHC[i] );
					RotPulseInProg[i]=true;
				}
			}
			else {
				degReqdRates.data[i] = 1000*sign( RHC[i] );
			}
		}
	}
	return outOfDetent;
}

void OrbitDAP::HandleTHCInput(double simdt)
{
	int THC[3];
	THC[0] = pTHC_SOP->GetXCommand();
	THC[1] = pTHC_SOP->GetYCommand();
	THC[2] = pTHC_SOP->GetZCommand();

	for(int i=0;i<3;i++) {
		if (THC[i] != 0)
		{
			if(TransMode[i]==NORM) {
				TransThrusterCommands[i].SetLine(static_cast<float>( sign( THC[i] ) ));
			}
			else if(TransMode[i]==TRANS_PULSE && !TransPulseInProg[i]) {
				TransPulseInProg[i]=true;
				TransPulseDV.data[i] = TransPulse*sign( THC[i] );
			}
			else {
				TransThrusterCommands[i].ResetLine();
			}
		}
		else {
			TransThrusterCommands[i].ResetLine();
		}

		if(TransPulseInProg[i]) {
			if(!Eq(TransPulseDV.data[i], 0.0, 0.001)) {
				TransThrusterCommands[i].SetLine(static_cast<float>( sign(TransPulseDV.data[i]) ));
				// calculate DV so far
				VECTOR3 ThrustVector;
				STS()->GetThrustVector(ThrustVector);
				ThrustVector = _V(ThrustVector.z, ThrustVector.x, -ThrustVector.y); // convert from Orbiter frame to shuttle frame
				double acc = (ThrustVector.data[i]/OrbiterMass)*simdt;
				if(abs(TransPulseDV.data[i]) <= abs(acc/2.0)) { //minimize error
					TransPulseDV.data[i]=0.000;
				}
				else {
					TransPulseDV.data[i] -= acc;
				}
			}
			else {
				//if THC is in detent and pulse is complete, allow further pulses
				if (THC[i] == 0) TransPulseInProg[i]=false;
				TransThrusterCommands[i].ResetLine();
			}
		}
	}
}

void OrbitDAP::CalcEulerAxisRates()
{
	double timeAcc = rcsmath::ThrustReduction( oapiGetTimeAcceleration(), STS()->GetOptions()->RCSFullThrustAcceleration() );// 1 up to the time acceleration of the option

	VECTOR3 RotationAxis;
	double RotationAngle=CalcEulerAngle(attErrorMatrix, RotationAxis);
	//Rates=RotationAxis*-RotRate;
	RotationAxis = _V(RotationAxis.y, RotationAxis.z, RotationAxis.x); // change rotation axis so PYR axes are mapped correctly
	for(unsigned int i=0;i<3;i++) {
		if(DAPControlMode==AUTO || RotMode[i]==DISC_RATE) {
			degReqdRates.data[i]=RotationAxis.data[i]*degRotRate;
			if(abs(ATT_ERR.data[i]) <= NullStartAngle(abs(radAngularVelocity.data[i]), OrbiterMass, PMI.data[i], Torque.data[i]/timeAcc)) {
				degReqdRates.data[i] = 0.0;
				RotatingAxis[i] = false;
				NullingRates[i] = true;
			}
			else RotatingAxis[i] = true;
		}
		else { // PULSE mode; maintain current rate
			degReqdRates.data[i] = degAngularVelocity.data[i];
		}
	}
}

void OrbitDAP::CalcMultiAxisRates(const VECTOR3& degNullRatesLocal)
{
	double timeAcc = rcsmath::ThrustReduction( oapiGetTimeAcceleration(), STS()->GetOptions()->RCSFullThrustAcceleration() );// 1 up to the time acceleration of the option

	for(unsigned int i=0;i<3;i++) {
		if(DAPControlMode==AUTO || RotMode[i]==DISC_RATE) {
			degReqdRates.data[i]=0.0;
			if((RotatingAxis[i] || abs(ATT_ERR.data[i])>degAttDeadband)) {
				if(abs(ATT_ERR.data[i])<0.05) {
					RotatingAxis[i]=false;
					NullingRates[i] = true;
				}
				else {
					RotatingAxis[i]=true;
					if(abs(ATT_ERR.data[i]) <= NullStartAngle(abs(radAngularVelocity.data[i]), OrbiterMass, PMI.data[i], Torque.data[i]/timeAcc)) {
						degReqdRates.data[i] = 0.0;
					}
					else {
						degReqdRates.data[i] = sign(ATT_ERR.data[i])*range(0.05*degRotRate, abs(ATT_ERR.data[i])/5.0, 0.1*degRotRate);
					}
				}
			}
		}
		else { // PULSE mode; maintain current rate
			degReqdRates.data[i] = degAngularVelocity.data[i];
		}
	}

	// add null rates to maintain attitude in rotation frame
	for(int i=0;i<3;i++) {
		if(DAPControlMode==AUTO || RotMode[i]==DISC_RATE) degReqdRates.data[i]+=degNullRatesLocal.data[i];
	}
}

void OrbitDAP::SetRates(const VECTOR3 &degRates, double simdt)
{
	const VECTOR3 PRI_LIMITS = _V(0.005, 0.005, 0.005);
	const VECTOR3 VERN_LIMITS = _V(0.0015, 0.0015, 0.0015);
	//static char buf[100];


	//double dDiff;
	VECTOR3 Error = degRates-degAngularVelocity;
	/*Error.data[YAW] = Error.data[YAW]; // temporary
	Error.data[ROLL] = Error.data[ROLL];*/



	VECTOR3 Limits;
	double MaxThrusterLevel;
	double timeAcc = rcsmath::ThrustReduction( oapiGetTimeAcceleration(), STS()->GetOptions()->RCSFullThrustAcceleration() );// 1 up to the time acceleration of the option
	if(DAPMode != VERN) { // PRI/ALT
		for(unsigned int i=0;i<3;i++) {
			//if(RotatingAxis[i] || NullingRates[i]) Limits.data[i] = max(PRI_LIMITS.data[i], 0.5*RotationRateChange(OrbiterMass, PMI.data[i], Torque.data[i]/timeAcc, simdt));
			if(RotatingAxis[i]) Limits.data[i] = max(2.0*PRI_LIMITS.data[i], 0.5*RotationRateChange(OrbiterMass, PMI.data[i], Torque.data[i]/timeAcc, simdt));
			else if(NullingRates[i] || RotPulseInProg[i]) Limits.data[i] = max(PRI_LIMITS.data[i], 0.5*RotationRateChange(OrbiterMass, PMI.data[i], Torque.data[i]/timeAcc, simdt));
			else Limits.data[i] = degRateDeadband;
		}
		MaxThrusterLevel = 1.0/timeAcc;
	}
	else { // VERN
		for(unsigned int i=0;i<3;i++) {
			if(RotatingAxis[i] || NullingRates[i]) Limits.data[i] = max(VERN_LIMITS.data[i], 0.5*RotationRateChange(OrbiterMass, PMI.data[i], 0.1*Torque.data[i]/timeAcc, simdt));
			else Limits.data[i] = degRateDeadband;
		}
		MaxThrusterLevel = 0.1/timeAcc;
	}
	//if(ManeuverStatus==MNVR_IN_PROGRESS) Limits=Limits*5.0;

	bool RHCdetent[3];
	RHCdetent[0] = pRHC_SOP->GetPitchDetent();
	RHCdetent[1] = pRHC_SOP->GetYawDetent();
	RHCdetent[2] = pRHC_SOP->GetRollDetent();

	for(unsigned int i=0;i<3;i++) {
		if(abs(Error.data[i])>Limits.data[i]) {
			//RotThrusterCommands[i].SetLine(static_cast<float>(MaxThrusterLevel*sign(Error.data[i])));
			double thrusterLevel = MaxThrusterLevel;
			if(DAPMode != VERN) { // for PRI/ALT, there are multiple thrusters in each direction, and we don't need to fire all of them
				double scale = abs(Error.data[i])/Limits.data[i];
				if(scale < 2) thrusterLevel = MaxThrusterLevel/3.0;
				else if(scale < 5) thrusterLevel = MaxThrusterLevel*(0.667);
			}
			RotThrusterCommands[i].SetLine(static_cast<float>(thrusterLevel*sign(Error.data[i])));
		}
		else {
			RotThrusterCommands[i].ResetLine();
			NullingRates[i] = false;
			//If RHC is out of detent, pretend pulse is still in progress
			if (RHCdetent[i] == true) RotPulseInProg[i]=false;
		}
	}




}

void OrbitDAP::OMSTVC(const VECTOR3 &AttErr, double SimDT)
{
	VECTOR3 biasedAttErr = AttErr + OMSAttBias;

	double pitchDelta = (biasedAttErr.data[PITCH] * 0.1) - degAngularVelocity.data[PITCH]; //if positive, vessel is pitching down
	double yawDelta = (biasedAttErr.data[YAW] * 0.1) - degAngularVelocity.data[YAW]; //if positive, vessel is rotating to right
	double rollDelta = (biasedAttErr.data[ROLL] * 0.1) - degAngularVelocity.data[ROLL]; //if positive, vessel is rolling to left

	bool RCSWraparound_Pitch = (fabs( biasedAttErr.data[PITCH] ) > 10.0);
	bool RCSWraparound_Yaw = (fabs( biasedAttErr.data[YAW] ) > 10.0);
	bool RCSWraparound_Roll = (fabs( biasedAttErr.data[ROLL] ) > 5.0);

	double dPitch = OMSTVCControlP.Step( -pitchDelta, SimDT );
	double dYaw = OMSTVCControlY.Step( -yawDelta, SimDT );
	double dRoll = OMSTVCControlR.Step( -rollDelta, SimDT );

	double Ltrim = 0.0;
	double Rtrim = 0.0;
	if(ControlMode!=RIGHT_OMS) //left OMS engine burning
	{
		double Pitch = dPitch + OMSTrim.data[0], Yaw = dYaw + OMSTrim.data[1];
		if(ControlMode==BOTH_OMS) Pitch-=dRoll;

		Ltrim = - dYaw;

		WriteCOMPOOL_VS( SCP_OMSL_PITCH_YAW_CMD, 1, static_cast<float>(range( -5.0, Pitch, 5.0 )), 2 ); 
		WriteCOMPOOL_VS( SCP_OMSL_PITCH_YAW_CMD, 2, static_cast<float>(range( -8.0, Yaw, 8.0 )), 2 ); 
	}
	if(ControlMode!=LEFT_OMS) //right OMS engine burning
	{
		double Pitch = dPitch + OMSTrim.data[0], Yaw = dYaw + OMSTrim.data[2];
		if(ControlMode==BOTH_OMS) Pitch+=dRoll;

		Rtrim = dYaw;
		if (ControlMode == BOTH_OMS) Rtrim = -Rtrim;

		WriteCOMPOOL_VS( SCP_OMSR_PITCH_YAW_CMD, 1, static_cast<float>(range( -5.0, Pitch, 5.0 )), 2 ); 
		WriteCOMPOOL_VS( SCP_OMSR_PITCH_YAW_CMD, 2, static_cast<float>(range( -8.0, Yaw, 8.0 )), 2 ); 
	}

	OMSAttBias = _V( -dPitch, Ltrim - Rtrim, -dRoll );// for next step

	double RCSpitch = 0.0;
	double RCSyaw = 0.0;
	double RCSroll = 0.0;

	if (RCSWraparound_Pitch) RCSpitch = biasedAttErr.data[PITCH] * 0.1;
	if (RCSWraparound_Yaw) RCSyaw = biasedAttErr.data[YAW] * 0.1;
	if ((ControlMode != BOTH_OMS) || (RCSWraparound_Roll)) RCSroll = biasedAttErr.data[ROLL] * 0.1;// for single-engine burns, use RCS for roll control

	// only fire the needed axis
	SetRates( _V( RCSpitch, RCSyaw, RCSroll ), SimDT );
	if (RCSpitch == 0.0) RotThrusterCommands[0].ResetLine();
	if (RCSyaw == 0.0) RotThrusterCommands[1].ResetLine();
	if (RCSroll == 0.0) RotThrusterCommands[2].ResetLine();
	return;
}

void OrbitDAP::GetAttitudeData()
{
	STS()->GetAngularVel(radAngularVelocity);
	radAngularVelocity = _V(radAngularVelocity.x, -radAngularVelocity.y, radAngularVelocity.z); // convert from Orbitersim to body axis frame
	degAngularVelocity = radAngularVelocity*DEG;
	RATE_EST = _V(radAngularVelocity.z, radAngularVelocity.x, -radAngularVelocity.y) * DEG;

	STS()->GetRotationMatrix(curM50Matrix);
	curM50Matrix = ConvertOrbitersimRotationMatrixToM50(curM50Matrix);

	VECTOR3 QV;
	double QS;
	MAT_TO_QUAT(Transpose(curM50Matrix), QS, QV);

	WriteCOMPOOL_VS(SCP_Q_B_I, 1, static_cast<float>(QS), 4);
	WriteCOMPOOL_VS(SCP_Q_B_I, 2, static_cast<float>(QV.x), 4);
	WriteCOMPOOL_VS(SCP_Q_B_I, 3, static_cast<float>(QV.y), 4);
	WriteCOMPOOL_VS(SCP_Q_B_I, 4, static_cast<float>(QV.z), 4);

	OrbiterMass = STS()->GetMass();
	STS()->GetPMI(PMI);
}

void OrbitDAP::Realize()
{
	DiscreteBundle* pBundle=BundleManager()->CreateBundle("THRUSTER_CMD", 16);
	for(unsigned int i=0;i<3;i++) {
		RotThrusterCommands[i].Connect(pBundle, i);
		TransThrusterCommands[i].Connect(pBundle, i+3);
	}

	pStateVector = dynamic_cast<StateVectorSoftware*>(FindSoftware("StateVectorSoftware"));
	assert( (pStateVector != NULL) && "OrbitDAP::Realize.pStateVector" );
	pUniversalPointing = dynamic_cast<UniversalPointing*>(FindSoftware("UniversalPointing"));
	assert((pUniversalPointing != NULL) && "OrbitDAP::Realize.pUniversalPointing");
	pRHC_SOP = dynamic_cast<RHC_SOP*>(FindSoftware( "RHC_SOP" ));
	assert( (pRHC_SOP != NULL) && "OrbitDAP::Realize.pRHC_SOP" );
	pTHC_SOP = dynamic_cast<THC_SOP*>(FindSoftware( "THC_SOP" ));
	assert( (pTHC_SOP != NULL) && "OrbitDAP::Realize.pTHC_SOP" );

	UpdateDAPParameters();
	GetAttitudeData();
}

void OrbitDAP::OnPreStep(double simt, double simdt, double mjd)
{
	unsigned short CDR_SPDBK_THROT = ReadCOMPOOL_IS( SCP_LH_SPD_BK_THROT_AUTO_MAN );
	unsigned short PLT_SPDBK_THROT = ReadCOMPOOL_IS( SCP_RH_SPD_BK_THROT_AUTO_MAN );
	unsigned short CDR_BodyFlap = ReadCOMPOOL_IS( SCP_LH_BODY_FLAP_AUTO_MANUAL );
	unsigned short blank_pbi_fwd = ReadCOMPOOL_IS( SCP_FWD_TRANSLATION_X_LVLH );
	unsigned short blank_pbi_aft = ReadCOMPOOL_IS( SCP_AFT_TRANSLATION_X_LVLH );
	cdrspdbkthrot.Set( CDR_SPDBK_THROT == 1 );
	pltspdbkthrot.Set( PLT_SPDBK_THROT == 1 );
	cdrbodyflap.Set( CDR_BodyFlap == 1 );
	sparepbi.Set( (blank_pbi_fwd == 1) || (blank_pbi_aft == 1) );

	GetAttitudeData();

	// PBI input
	DAP_PBI_Press();

	// DAP lights command
	CommandDAPLights();

	// PCT status lights
	if (PCTArmed)
	{
		// both SPD BK AUTO LTs on
		WriteCOMPOOL_IS( SCP_FF1_IOM10_CH0_DATA, ReadCOMPOOL_IS( SCP_FF1_IOM10_CH0_DATA ) | 0x0020 );
		WriteCOMPOOL_IS( SCP_FF3_IOM10_CH0_DATA, ReadCOMPOOL_IS( SCP_FF3_IOM10_CH0_DATA ) | 0x0020 );

		if (PCTActive)
		{
			// CDR Body Flap and spare DAP LTs on
			WriteCOMPOOL_IS( SCP_FF1_IOM10_CH0_DATA, ReadCOMPOOL_IS( SCP_FF1_IOM10_CH0_DATA ) | 0x0010 );
			WriteCOMPOOL_IS( SCP_FF2_IOM10_CH0_DATA, ReadCOMPOOL_IS( SCP_FF2_IOM10_CH0_DATA ) | 0x0010 );
			// spare DAP done in CommandDAPLights()
		}
	}

	// in AUTO, INRTL or LVLH modes, start maneuver
	if(bFirstStep) {
		if(DAPControlMode == AUTO) {
			if(CurManeuver.IsValid) StartCurManeuver();
		}
		else if(DAPControlMode == INRTL) {
			StartManeuver(curM50Matrix, AttManeuver::MNVR);
		}
		else if(DAPControlMode == LVLH) {
			StartManeuver(GetCurrentLVLHAttMatrix(), AttManeuver::LVLH_TRK);
		}
		bFirstStep = false;
	}

	// PCT
	if ((GetMajorMode() / 100) == 2)
	{
		if (PCTArmed)
		{
			if (PCTActive)
			{
				// active
				if (cdrspdbkthrot.Get() || pltspdbkthrot.Get())
				{
					DisarmPCT();
					StopPCT();
				}
				else
				{
					bool RHCdetent = pRHC_SOP->GetPitchDetent() &&
						pRHC_SOP->GetYawDetent() &&
						pRHC_SOP->GetRollDetent();

					bool THCdetent = (pTHC_SOP->GetXCommand() == 0) &&
						(pTHC_SOP->GetYCommand() == 0) &&
						(pTHC_SOP->GetZCommand() == 0);

					unsigned short FWD_DAP_FREE = ReadCOMPOOL_IS( SCP_FWD_RCS_OPTIONS_VERNIER );
					unsigned short AFT_DAP_FREE = ReadCOMPOOL_IS( SCP_AFT_RCS_OPTIONS_VERNIER );
					if (cdrbodyflap.Get() || sparepbi.Get() || (FWD_DAP_FREE == 1) || (AFT_DAP_FREE == 1) || !RHCdetent || !THCdetent) StopPCT();
				}
			}
			else
			{
				// armed
				if (cdrspdbkthrot.Get() || pltspdbkthrot.Get()) DisarmPCT();
				else if (cdrbodyflap.Get() || sparepbi.Get()) StartPCT();
			}
		}
		else
		{
			// disarmed
			if (cdrspdbkthrot.Get() || pltspdbkthrot.Get()) ArmPCT();
		}

		if (PCTActive == true) PCTControl( simt );
		else HandleTHCInput( simdt );
	}
	else HandleTHCInput(simdt);

	if(GetRHCRequiredRates()) {
		if(DAPControlMode == AUTO) {
			DAPControlMode = INRTL;
			StartManeuver(curM50Matrix, AttManeuver::MNVR);
		}
		else if(DAPControlMode == INRTL) {
			StartManeuver(curM50Matrix, AttManeuver::MNVR);
		}
		else if(DAPControlMode == LVLH) {
			StartManeuver(GetCurrentLVLHAttMatrix(), AttManeuver::LVLH_TRK);
		}
		ATT_ERR = _V(0.0, 0.0, 0.0);
	}
	else if(DAPControlMode != FREE) { // if DAP is in FREE, we only care about RHC input; otherwise, we want to maintain target attitude
		MATRIX3 tgtM50Matrix;  // target M50 attitude for this timestep
		if (ActiveManeuver.Type == AttManeuver::UP)
		{
			// Universal Pointing

			double Q_B_M50_DESIRED_S;
			VECTOR3 Q_B_M50_DESIRED_V, DESIRED_BODY_RATE;

			pUniversalPointing->GetRequiredQuaternion(Q_B_M50_DESIRED_S, Q_B_M50_DESIRED_V);
			DESIRED_BODY_RATE = pUniversalPointing->Get_REQD_BRATE();

			tgtM50Matrix = QUAT_TO_MAT(Q_B_M50_DESIRED_S, -Q_B_M50_DESIRED_V);
			degNullRates = _V(DESIRED_BODY_RATE.y, DESIRED_BODY_RATE.z, DESIRED_BODY_RATE.x); // For now convert to Orbiter axes
		}
		else if(ActiveManeuver.Type == AttManeuver::LVLH_TRK) { // get (instantaneous) target M50 attitude
			MATRIX3 curLVLHMatrix = GetCurrentLVLHRefMatrix();
			tgtM50Matrix = mul(curLVLHMatrix, ActiveManeuver.tgtMatrix);

			// Required body rate for Orbit DAP LVLH mode also comes from Universal Pointing
			VECTOR3 DESIRED_BODY_RATE = pUniversalPointing->Get_REQD_BRATE();
			degNullRates = _V(DESIRED_BODY_RATE.y, DESIRED_BODY_RATE.z, DESIRED_BODY_RATE.x); // For now convert to Orbiter axes

			if((STS()->GetMET()-lastUpdateTime) > 60.0) {
				if(ManeuverStatus < MNVR_COMPLETE) mnvrCompletionMET = STS()->GetMET() + CalcManeuverCompletionTime(curM50Matrix, ActiveManeuver.tgtMatrix, curLVLHMatrix, length(degNullRates));
				lastUpdateTime = STS()->GetMET();
			}
			if(ManeuverStatus == MNVR_IN_PROGRESS) {
				// calculate M50 matrix corresponding to target attitude at end of maneuver
				double rotationAngle = RAD*length(degNullRates)*(mnvrCompletionMET-STS()->GetMET());
				MATRIX3 LVLHRotation;
				GetRotMatrixY(-rotationAngle, LVLHRotation);
				tgtM50Matrix = mul(mul(curLVLHMatrix, LVLHRotation), ActiveManeuver.tgtMatrix);
			}
		}
		else {
			tgtM50Matrix = ActiveManeuver.tgtMatrix;

			if (((STS()->GetMET()-lastUpdateTime) > 10.0) && (ManeuverStatus < MNVR_COMPLETE))
			{
				// recalculate time to reach target attitude
				VECTOR3 Axis;
				MATRIX3 AttError = GetRotationErrorMatrix(curM50Matrix, ActiveManeuver.tgtMatrix);
				double Angle = CalcEulerAngle(AttError, Axis);
				mnvrCompletionMET = STS()->GetMET() + (Angle*DEG)/degRotRate;
				lastUpdateTime = STS()->GetMET();
			}
		}

		attErrorMatrix = GetRotationErrorMatrix(curM50Matrix, tgtM50Matrix);
		ATT_ERR=GetYZX_PYRAnglesFromMatrix(attErrorMatrix)*DEG;
		if(ManeuverStatus == MNVR_COMPLETE) {
			CalcMultiAxisRates(degNullRates);
		}
		else if(ManeuverStatus == MNVR_IN_PROGRESS) {
			CalcEulerAxisRates();
			if(!RotatingAxis[PITCH] && !RotatingAxis[YAW] && !RotatingAxis[ROLL]) {
				ManeuverStatus=MNVR_COMPLETE; //now maintaining targ. attitude
			}
		}
		else { // MNVR_STARTING
			degReqdRates = _V(0.0, 0.0, 0.0);
		}
	}
	else { // FREE
		degReqdRates = degAngularVelocity;
	}
	if(ControlMode == RCS) SetRates(degReqdRates, simdt);
	else OMSTVC(ATT_ERR, simdt);

	lastStepdt = simdt;
}

bool OrbitDAP::OnMajorModeChange(unsigned int newMajorMode)
{
	if ((newMajorMode >= 104) && (newMajorMode <= 303))
	{
		return true;
	}
	return false;
}

bool OrbitDAP::ItemInput( int item, const char* Data )
{
	int profile = GetCurrentProfile(item);

	switch (item)
	{
		case 1:
			{
				int num;
				if (GetIntegerUnsigned(Data, num))
				{
					if (num > 0 && num <= 15)
					{
						activeAProfile = num-1;
					}
					else return false;
				}
				else return false;
			}
			break;
		case 2:
			{
				int num;
				if (GetIntegerUnsigned(Data, num))
				{
					if (num > 0 && num <= 15)
					{
						activeBProfile = num-1;
					}
					else return false;
				}
				else return false;
			}
		break;
		case 3:
		case 4:
			{
				int num;
				if (GetIntegerUnsigned( Data, num ))
				{
					if (num > 0 && num <= 15)
					{
						editDAP=item-3;
						editDAPProfile = num-1;
						DAPConfiguration[2][0] = DAPConfiguration[editDAP][num-1];
					}
					else return false;
				}
				else return false;
			}
			break;
		case 5:
			if (strlen( Data ) == 0)
			{
				if(editDAP != -1) DAPConfiguration[editDAP][editDAPProfile] = DAPConfiguration[2][0];
				editDAP=-1;
			}
			else return false;
			break;
		case 10:
		case 30:
		case 50:
			{
				double dNew;
				if (GetDoubleUnsigned( Data, dNew ))
				{
					if(dNew>=0.05 && dNew<=2.0)
					{
						DAPConfiguration[convert[item]][activeAProfile].PRI_ROT_RATE=dNew;
						if(convert[item]==DAPSelect && DAPMode==PRI) UpdateDAPParameters();
					}
					else return false;
				}
				else return false;
			}
			break;
		case 11:
		case 31:
		case 51:
			{
				double dNew;
				if (GetDoubleUnsigned( Data, dNew ))
				{
					if(dNew>0.10 && dNew<=40.0)
					{
						DAPConfiguration[convert[item]][GetCurrentProfile(item)].PRI_ATT_DB=dNew;
						if(convert[item]==DAPSelect && DAPMode==PRI) UpdateDAPParameters();
					}
					else return false;
				}
				else return false;
			}
			break;
		case 12:
		case 32:
		case 52:
			{
				double dNew;
				if (GetDoubleUnsigned( Data, dNew ))
				{
					if(dNew>=0.10 && dNew<=5.0)
					{
						DAPConfiguration[convert[item]][GetCurrentProfile(item)].PRI_RATE_DB=dNew;
						if(convert[item]==DAPSelect && DAPMode==PRI) UpdateDAPParameters();
					}
					else return false;
				}
				else return false;
			}
			break;
		case 13:
		case 33:
		case 53:
			{
				double dNew;
				if (GetDoubleUnsigned( Data, dNew ))
				{
					if(dNew>=0.04 && dNew<=1.0)
					{
						DAPConfiguration[convert[item]][GetCurrentProfile(item)].PRI_ROT_PLS=dNew;
						if(convert[item]==DAPSelect && DAPMode==PRI) UpdateDAPParameters();
					}
					else return false;
				}
				else return false;
			}
			break;
		case 14:
		case 34:
		case 54:
			{
				double dNew;
				if (GetDoubleUnsigned( Data, dNew ))
				{
					if (dNew<=0.999)
					{
						DAPConfiguration[convert[item]][GetCurrentProfile(item)].PRI_COMP=dNew;
						if(convert[item]==DAPSelect && DAPMode==PRI) UpdateDAPParameters();
					}
					else return false;
				}
				else return false;
			}
			break;
		case 15:
		case 35:
		case 55:
		{
			if (strlen(Data) == 0)
			{
				if (DAPConfiguration[convert[item]][profile].PRI_P_OPTION < 2)
				{
					DAPConfiguration[convert[item]][profile].PRI_P_OPTION++;
					if (DAPMode == PRI)
					{
						if (DAPConfiguration[DAPSelect][profile].PRI_P_OPTION == 1)
						{
							STS()->DisableThrusters(AftPitchThrusters, 2);
							UpdateDAPParameters();
						}
						else if (DAPConfiguration[DAPSelect][profile].PRI_P_OPTION == 2)
						{
							STS()->EnableThrusters(AftPitchThrusters, 2);
							STS()->DisableThrusters(NosePitchThrusters, 2);
							UpdateDAPParameters();
						}
					}
				}
				else
				{
					DAPConfiguration[convert[item]][profile].PRI_P_OPTION = 0;
					if (DAPConfiguration[DAPSelect][profile].PRI_P_OPTION == 0)
					{
						STS()->EnableThrusters(NosePitchThrusters, 2);
						UpdateDAPParameters();
					}
				}
			}
			else return false;
			break;
		}
		case 16:
		case 36:
		case 56:
			if (strlen( Data ) == 0)
			{
				if (DAPConfiguration[convert[item]][profile].PRI_Y_OPTION<2)
				{
					DAPConfiguration[convert[item]][profile].PRI_Y_OPTION++;
					if (DAPMode==PRI)
					{
						if (DAPConfiguration[DAPSelect][profile].PRI_Y_OPTION==1)
						{
							STS()->DisableThrusters(AftYawThrusters, 2);
							UpdateDAPParameters();
						}
						else if (DAPConfiguration[DAPSelect][profile].PRI_Y_OPTION==2)
						{
							STS()->EnableThrusters(AftYawThrusters, 2);
							STS()->DisableThrusters(NoseYawThrusters, 2);
							UpdateDAPParameters();
						}
					}
				}
				else
				{
					DAPConfiguration[convert[item]][profile].PRI_Y_OPTION=0;
					if (DAPConfiguration[DAPSelect][profile].PRI_Y_OPTION==0)
					{
						STS()->EnableThrusters(NoseYawThrusters, 2);
						UpdateDAPParameters();
					}
				}
			}
			else return false;
			break;
		case 17:
		case 37:
		case 57:
			{
				double dNew;
				if (GetDoubleUnsigned( Data, dNew ))
				{
					if (dNew>=0.01 && dNew<=5.0)
					{
						DAPConfiguration[convert[item]][profile].PRI_TRAN_PLS=dNew;
						if (convert[item]== DAPSelect && DAPMode==PRI) UpdateDAPParameters();
					}
					else return false;
				}
				else return false;
			}
			break;
		case 18:
		case 38:
		case 58:
			{
				double dNew;
				if (GetDoubleUnsigned( Data, dNew ))
				{
					if (dNew>=0.05 && dNew<=5.0)
					{
						DAPConfiguration[convert[item]][profile].ALT_RATE_DB=dNew;
						if (convert[item]==DAPSelect && DAPMode==ALT) UpdateDAPParameters();
					}
					else return false;
				}
				else return false;
			}
			break;
		case 19:
		case 39:
		case 59:
			if (strlen( Data ) == 0)
			{
				if (DAPConfiguration[convert[item]][profile].ALT_JET_OPT==0)
				{
					DAPConfiguration[convert[item]][profile].ALT_JET_OPT=2;
					if (DAPMode==ALT)
					{
						if (DAPConfiguration[DAPSelect][profile].ALT_JET_OPT==2)
						{
							STS()->DisableThrusters(NoseRotThrusters, 6);
							STS()->EnableThrusters(AftRotThrusters, 6);
							UpdateDAPParameters();
						}
					}
				}
				else
				{
					DAPConfiguration[convert[item]][profile].ALT_JET_OPT=0;
					if (DAPMode==ALT)
					{
						if (DAPConfiguration[DAPSelect][profile].ALT_JET_OPT==0)
						{
							STS()->EnableThrusters(NoseRotThrusters, 6);
							UpdateDAPParameters();
						}
					}
				}
			}
			else return false;
			break;
		case 20:
		case 40:
		case 60:
			{
				int nNew;
				if (GetIntegerUnsigned( Data, nNew ))
				{
					if (nNew>=1 && nNew<=3)
					{
						DAPConfiguration[convert[item]][profile].ALT_JETS=nNew;
						if (convert[item]==DAPSelect && DAPMode==ALT) UpdateDAPParameters();
					}
					else return false;
				}
				else return false;
			}
			break;
		case 21:
		case 41:
		case 61:
			{
				double dNew;
				if (GetDoubleUnsigned( Data, dNew ))
				{
					if (dNew>=0.08 && dNew<=9.99)
					{
						DAPConfiguration[convert[item]][profile].ALT_ON_TIME=dNew;
						if (convert[item]==DAPSelect && DAPMode==ALT) UpdateDAPParameters();
					}
					else return false;
				}
				else return false;
			}
			break;
		case 22:
		case 42:
		case 62:
			{
				double dNew;
				if (GetDoubleUnsigned( Data, dNew ))
				{
					if (dNew<=99.99)
					{
						DAPConfiguration[convert[item]][profile].ALT_DELAY=dNew;
						if (convert[item]==DAPSelect && DAPMode==ALT) UpdateDAPParameters();
					}
					else return false;
				}
				else return false;
			}
			break;
		case 23:
		case 43:
		case 63:
			{
				double dNew;
				if (GetDoubleUnsigned( Data, dNew ))
				{
					if (dNew>=0.002 && dNew<=1.0)
					{
						DAPConfiguration[convert[item]][profile].VERN_ROT_RATE=dNew;
						if (convert[item]==DAPSelect && DAPMode==VERN) UpdateDAPParameters();
					}
					else return false;
				}
				else return false;
			}
			break;
		case 24:
		case 44:
		case 64:
			{
				double dNew;
				if (GetDoubleUnsigned( Data, dNew ))
				{
					if (dNew>=0.01 && dNew<=40.0)
					{
						DAPConfiguration[convert[item]][profile].VERN_ATT_DB=dNew;
						if (convert[item]==DAPSelect && DAPMode==VERN) UpdateDAPParameters();
					}
					else return false;
				}
				else return false;
			}
			break;
		case 25:
		case 45:
		case 65:
			{
				double dNew;
				if (GetDoubleUnsigned( Data, dNew ))
				{
					if (dNew>=0.01 && dNew<=0.5)
					{
						DAPConfiguration[convert[item]][profile].VERN_RATE_DB=dNew;
						if (convert[item]==DAPSelect && DAPMode==VERN) UpdateDAPParameters();
					}
					else return false;
				}
				else return false;
			}
			break;
		case 26:
		case 46:
		case 66:
			{
				double dNew;
				if (GetDoubleUnsigned( Data, dNew ))
				{
					if (dNew>=0.001 && dNew<=0.5)
					{
						DAPConfiguration[convert[item]][profile].VERN_ROT_PLS=dNew;
						if (convert[item]==DAPSelect && DAPMode==VERN) UpdateDAPParameters();
					}
					else return false;
				}
				else return false;
			}
			break;
		case 27:
		case 47:
		case 67:
			{
				double dNew;
				if (GetDoubleUnsigned( Data, dNew ))
				{
					if (dNew<=0.999)
					{
						DAPConfiguration[convert[item]][profile].VERN_COMP=dNew;
						if (convert[item]==DAPSelect && DAPMode==VERN) UpdateDAPParameters();
					}
					else return false;
				}
				else return false;
			}
			break;
		case 28:
		case 48:
		case 68:
			{
				int nNew;
				if (GetIntegerUnsigned( Data, nNew ))
				{
					if (nNew<=9)
					{
						DAPConfiguration[convert[item]][profile].VERN_CNTL_ACC=nNew;
						if (convert[item]==DAPSelect && DAPMode==VERN) UpdateDAPParameters();
					}
					else return false;
				}
				else return false;
			}
			break;
		default:
			return false;
	}
	return true;
}

void OrbitDAP::OnPaint(vc::MDU* pMDU) const
{
	char *strings[3]={" ALL", "NOSE", "TAIL"};
	char cbuf[255];
	int lim[3]={3, 5, 5};
	int i, n;

	PrintCommonHeader("   DAP CONFIG", pMDU);

	pMDU->mvprint(4, 2, "PRI");
	pMDU->mvprint(9, 2, "1 DAP A");
	pMDU->mvprint(20, 2, "2 DAP B");
	pMDU->mvprint(33, 2, "PRI");
	pMDU->mvprint(0, 3, "ROT RATE");
	pMDU->mvprint(0, 4, "ATT DB");
	pMDU->mvprint(0, 5, "RATE DB");
	pMDU->mvprint(0, 6, "ROT PLS");
	pMDU->mvprint(0, 7, "COMP");
	pMDU->mvprint(0, 8, "P OPTION");
	pMDU->mvprint(0, 9, "Y OPTION");
	pMDU->mvprint(0, 10, "TRAN PLS");

	pMDU->mvprint(4, 11, "ALT");
	pMDU->mvprint(33, 11, "ALT");
	pMDU->mvprint(0, 12, "RATE DB");
	pMDU->mvprint(0, 13, "JET OPT");
	pMDU->mvprint(0, 14, "# JETS");
	pMDU->mvprint(0, 15, "ON TIME");
	pMDU->mvprint(0, 16, "DELAY");

	pMDU->mvprint(4, 17, "VERN");
	pMDU->mvprint(33, 17, "VERN");
	pMDU->mvprint(0, 18, "ROT RATE");
	pMDU->mvprint(0, 19, "ATT DB");
	pMDU->mvprint(0, 20, "RATE DB");
	pMDU->mvprint(0, 21, "ROT PLS");
	pMDU->mvprint(0, 22, "COMP");
	pMDU->mvprint(0, 23, "CNTL ACC");


	sprintf_s(cbuf, 255, "%02d", activeAProfile+1);
	pMDU->mvprint( 16, 2, cbuf );
	pMDU->Underline( 16, 2 );
	pMDU->Underline( 17, 2 );
	sprintf_s(cbuf, 255, "%02d", activeBProfile+1);
	pMDU->mvprint( 27, 2, cbuf );
	pMDU->Underline( 27, 2 );
	pMDU->Underline( 28, 2 );

	int edit=2; //temporary
	for(n=1, i=0;n<=lim[edit];n+=2, i++) {
		int profile;
		if (i == 0)
			profile = activeAProfile;
		else if (i == 1)
			profile = activeBProfile;
		else
			profile = 0;

		sprintf_s(cbuf, 255, "%d %.4f", 10*n, DAPConfiguration[i][profile].PRI_ROT_RATE);
		pMDU->mvprint(9+11*i, 3, cbuf);
		pMDU->Underline( 12 + 11 * i, 3 );
		pMDU->Underline( 13 + 11 * i, 3 );
		pMDU->Underline( 14 + 11 * i, 3 );
		pMDU->Underline( 15 + 11 * i, 3 );
		pMDU->Underline( 16 + 11 * i, 3 );
		pMDU->Underline( 17 + 11 * i, 3 );
		sprintf_s(cbuf, 255, "%d  %05.2f", 10*n+1, DAPConfiguration[i][profile].PRI_ATT_DB);
		pMDU->mvprint(9+11*i, 4, cbuf);
		pMDU->Underline( 13 + 11 * i, 4 );
		pMDU->Underline( 14 + 11 * i, 4 );
		pMDU->Underline( 15 + 11 * i, 4 );
		pMDU->Underline( 16 + 11 * i, 4 );
		pMDU->Underline( 17 + 11 * i, 4 );
		sprintf_s(cbuf, 255, "%d   %.2f", 10*n+2, DAPConfiguration[i][profile].PRI_RATE_DB);
		pMDU->mvprint(9+11*i, 5, cbuf);
		pMDU->Underline( 14 + 11 * i, 5 );
		pMDU->Underline( 15 + 11 * i, 5 );
		pMDU->Underline( 16 + 11 * i, 5 );
		pMDU->Underline( 17 + 11 * i, 5 );
		sprintf_s(cbuf, 255, "%d  %.3f", 10*n+3, DAPConfiguration[i][profile].PRI_ROT_PLS);
		pMDU->mvprint(9+11*i, 6, cbuf);
		pMDU->Underline( 13 + 11 * i, 6 );
		pMDU->Underline( 14 + 11 * i, 6 );
		pMDU->Underline( 15 + 11 * i, 6 );
		pMDU->Underline( 16 + 11 * i, 6 );
		pMDU->Underline( 17 + 11 * i, 6 );
		sprintf_s(cbuf, 255, "%d  %.3f", 10*n+4, DAPConfiguration[i][profile].PRI_COMP);
		pMDU->mvprint(9+11*i, 7, cbuf);
		pMDU->mvprint( 13 + 11 * i, 7, " " );
		pMDU->Underline( 14 + 11 * i, 7 );
		pMDU->Underline( 15 + 11 * i, 7 );
		pMDU->Underline( 16 + 11 * i, 7 );
		pMDU->Underline( 17 + 11 * i, 7 );
		sprintf_s(cbuf, 255, "%d   %s", 10*n+5, strings[DAPConfiguration[i][profile].PRI_P_OPTION]);
		pMDU->mvprint(9+11*i, 8, cbuf);
		sprintf_s(cbuf, 255, "%d   %s", 10*n+6, strings[DAPConfiguration[i][profile].PRI_Y_OPTION]);
		pMDU->mvprint(9+11*i, 9, cbuf);
		sprintf_s(cbuf, 255, "%d  %.3f", 10*n+7, DAPConfiguration[i][profile].PRI_TRAN_PLS);
		pMDU->mvprint(9+11*i, 10, cbuf);
		pMDU->Underline( 13 + 11 * i, 10 );
		pMDU->Underline( 14 + 11 * i, 10 );
		pMDU->Underline( 15 + 11 * i, 10 );
		pMDU->Underline( 16 + 11 * i, 10 );
		pMDU->Underline( 17 + 11 * i, 10 );

		sprintf_s(cbuf, 255, "%d  %.3f", 10*n+8, DAPConfiguration[i][profile].ALT_RATE_DB);
		pMDU->mvprint(9+11*i, 12, cbuf);
		pMDU->Underline( 13 + 11 * i, 12 );
		pMDU->Underline( 14 + 11 * i, 12 );
		pMDU->Underline( 15 + 11 * i, 12 );
		pMDU->Underline( 16 + 11 * i, 12 );
		pMDU->Underline( 17 + 11 * i, 12 );
		sprintf_s(cbuf, 255, "%d   %s", 10*n+9, strings[DAPConfiguration[i][profile].ALT_JET_OPT]);
		pMDU->mvprint(9+11*i, 13, cbuf);
		sprintf_s(cbuf, 255, "%d      %d", 10*n+10, DAPConfiguration[i][profile].ALT_JETS);
		pMDU->mvprint(9+11*i, 14, cbuf);
		pMDU->Underline( 17 + 11 * i, 14 );
		sprintf_s(cbuf, 255, "%d   %.2f", 10*n+11, DAPConfiguration[i][profile].ALT_ON_TIME);
		pMDU->mvprint(9+11*i, 15, cbuf);
		pMDU->Underline( 14 + 11 * i, 15 );
		pMDU->Underline( 15 + 11 * i, 15 );
		pMDU->Underline( 16 + 11 * i, 15 );
		pMDU->Underline( 17 + 11 * i, 15 );
		sprintf_s(cbuf, 255, "%d  %05.2f", 10*n+12, DAPConfiguration[i][profile].ALT_DELAY);
		pMDU->mvprint(9+11*i, 16, cbuf);
		pMDU->Underline( 13 + 11 * i, 16 );
		pMDU->Underline( 14 + 11 * i, 16 );
		pMDU->Underline( 15 + 11 * i, 16 );
		pMDU->Underline( 16 + 11 * i, 16 );
		pMDU->Underline( 17 + 11 * i, 16 );

		sprintf_s(cbuf, 255, "%d %.4f", 10*n+13, DAPConfiguration[i][profile].VERN_ROT_RATE);
		pMDU->mvprint(9+11*i, 18, cbuf);
		pMDU->Underline( 12 + 11 * i, 18 );
		pMDU->Underline( 13 + 11 * i, 18 );
		pMDU->Underline( 14 + 11 * i, 18 );
		pMDU->Underline( 15 + 11 * i, 18 );
		pMDU->Underline( 16 + 11 * i, 18 );
		pMDU->Underline( 17 + 11 * i, 18 );
		sprintf_s(cbuf, 255, "%d %06.3f", 10*n+14, DAPConfiguration[i][profile].VERN_ATT_DB);
		pMDU->mvprint(9+11*i, 19, cbuf);
		pMDU->Underline( 12 + 11 * i, 19 );
		pMDU->Underline( 13 + 11 * i, 19 );
		pMDU->Underline( 14 + 11 * i, 19 );
		pMDU->Underline( 15 + 11 * i, 19 );
		pMDU->Underline( 16 + 11 * i, 19 );
		pMDU->Underline( 17 + 11 * i, 19 );
		sprintf_s(cbuf, 255, "%d  %.3f", 10*n+15, DAPConfiguration[i][profile].VERN_RATE_DB);
		pMDU->mvprint(9+11*i, 20, cbuf);
		pMDU->mvprint( 13 + 11 * i, 20, " " );
		pMDU->Underline( 14 + 11 * i, 20 );
		pMDU->Underline( 15 + 11 * i, 20 );
		pMDU->Underline( 16 + 11 * i, 20 );
		pMDU->Underline( 17 + 11 * i, 20 );
		sprintf_s(cbuf, 255, "%d  %05.3f", 10*n+16, DAPConfiguration[i][profile].VERN_ROT_PLS);
		pMDU->mvprint(9+11*i, 21, cbuf);
		pMDU->Underline( 13 + 11 * i, 21 );
		pMDU->Underline( 14 + 11 * i, 21 );
		pMDU->Underline( 15 + 11 * i, 21 );
		pMDU->Underline( 16 + 11 * i, 21 );
		pMDU->Underline( 17 + 11 * i, 21 );
		sprintf_s(cbuf, 255, "%d  %.3f", 10*n+17, DAPConfiguration[i][profile].VERN_COMP);
		pMDU->mvprint(9+11*i, 22, cbuf);
		pMDU->mvprint( 13 + 11 * i, 22, " " );
		pMDU->Underline( 14 + 11 * i, 22 );
		pMDU->Underline( 15 + 11 * i, 22 );
		pMDU->Underline( 16 + 11 * i, 22 );
		pMDU->Underline( 17 + 11 * i, 22 );
		sprintf_s(cbuf, 255, "%d      %d", 10*n+18, DAPConfiguration[i][profile].VERN_CNTL_ACC);
		pMDU->mvprint(9+11*i, 23, cbuf);
		pMDU->Underline( 17 + 11 * i, 23 );
	}

	pMDU->mvprint( 41, 2, "DAP EDIT" );
	pMDU->mvprint( 41, 3, "3 DAP A" );
	pMDU->mvprint( 41, 4, "4 DAP B" );
	pMDU->mvprint( 41, 5, "5" );

	pMDU->Line( 190, 28, 190, 336 );
	pMDU->Line( 300, 28, 300, 336 );
	pMDU->Line( 410, 112, 410, 336 );
	pMDU->Line( 410, 112, 510, 112 );

	pMDU->mvprint( 41, 9, "NOTCH FLTR" );
	pMDU->mvprint( 43, 10, "ENA  6" );

	pMDU->mvprint( 42, 12, "XJETS ROT" );
	pMDU->mvprint( 44, 13, "ENA  7" );

	pMDU->mvprint( 42, 15, "REBOOST" );
	pMDU->mvprint( 43, 16, "8 CFG" );
	pMDU->mvprint( 43, 17, "9 INTVL" );
}

bool OrbitDAP::OnParseLine(const char* keyword, const char* value)
{
	if(!_strnicmp(keyword, "DAP_MODE", 8)) {
		int nTemp1, nTemp2;
		sscanf_s(value, "%d %d", &nTemp1, &nTemp2);
		DAPSelect = static_cast<DAP_SELECT>(nTemp1);
		DAPMode = static_cast<DAP_MODE>(nTemp2);
		return true;
	}
	else if(!_strnicmp(keyword, "ROT_MODE", 8)) {
		int nTemp[3];
		sscanf_s(value, "%d %d %d", &nTemp[0], &nTemp[1], &nTemp[2]);
		for(int i=0;i<3;i++) RotMode[i] = static_cast<ROT_MODE>(nTemp[i]);
		return true;
	}
	else if(!_strnicmp(keyword, "TRANS_MODE", 10)) {
		int nTemp[3];
		sscanf_s(value, "%d %d %d", &nTemp[0], &nTemp[1], &nTemp[2]);
		for(int i=0;i<3;i++) TransMode[i] = static_cast<TRANS_MODE>(nTemp[i]);
		return true;
	}
	else if(!_strnicmp(keyword, "CONTROL_MODE", 12)) {
		//sscanf(value, "%d", &ControlMode);
		int nTemp;
		sscanf_s(value, "%d", &nTemp);
		if(nTemp==0) DAPControlMode=AUTO;
		else if(nTemp==1) DAPControlMode=INRTL;
		else if(nTemp==2) DAPControlMode=LVLH;
		else if(nTemp==3) {
			DAPControlMode=FREE;
			for(unsigned int i=0;i<3;i++) {
				RotMode[i]=ROT_PULSE;
				RotPulseInProg[i]=false;
			}
		}
		return true;
	}
	else if(!_strnicmp (keyword, "CUR_MNVR", 8)) {
		LoadAttManeuver(value, CurManeuver);
		return true;
	}

	return false;
}

void OrbitDAP::OnSaveState(FILEHANDLE scn) const
{
	char cbuf[256];
	sprintf_s(cbuf, 256, "%d %d", DAPSelect, DAPMode);
	oapiWriteScenario_string (scn, "DAP_MODE", cbuf);
	sprintf_s(cbuf, 256, "%d %d %d", RotMode[0], RotMode[1], RotMode[2]);
	oapiWriteScenario_string (scn, "ROT_MODE", cbuf);
	sprintf_s(cbuf, 256, "%d %d %d", TransMode[0], TransMode[1], TransMode[2]);
	oapiWriteScenario_string (scn, "TRANS_MODE", cbuf);
	sprintf_s(cbuf, 256, "%d", DAPControlMode);
	oapiWriteScenario_string (scn, "CONTROL_MODE", cbuf);
	if(CurManeuver.IsValid) SaveAttManeuver(scn, "CUR_MNVR", CurManeuver);
}

void OrbitDAP::CommandDAPLights( void )
{
	if (DAPSelect == A)
	{
		WriteCOMPOOL_IS( SCP_FWD_DAP_SELECT_A_LT, 1 );
		WriteCOMPOOL_IS( SCP_AFT_DAP_SELECT_A_LT, 1 );
		WriteCOMPOOL_IS( SCP_FWD_DAP_SELECT_B_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_DAP_SELECT_B_LT, 0 );
	}
	else// if (DAPSelect == B)
	{
		WriteCOMPOOL_IS( SCP_FWD_DAP_SELECT_A_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_DAP_SELECT_A_LT, 0 );
		WriteCOMPOOL_IS( SCP_FWD_DAP_SELECT_B_LT, 1 );
		WriteCOMPOOL_IS( SCP_AFT_DAP_SELECT_B_LT, 1 );
	}

	if (DAPControlMode == AUTO)
	{
		WriteCOMPOOL_IS( SCP_FWD_DAP_AUTO_LT, 1 );
		WriteCOMPOOL_IS( SCP_AFT_DAP_AUTO_LT, 1 );
		WriteCOMPOOL_IS( SCP_FWD_DAP_MAN_INRTL_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_DAP_MAN_INRTL_LT, 0 );
		WriteCOMPOOL_IS( SCP_FWD_DAP_MAN_LVLH_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_DAP_MAN_LVLH_LT, 0 );
		WriteCOMPOOL_IS( SCP_FWD_DAP_MAN_FREE_DFT_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_DAP_MAN_FREE_DFT_LT, 0 );
	}
	else if (DAPControlMode == INRTL)
	{
		WriteCOMPOOL_IS( SCP_FWD_DAP_AUTO_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_DAP_AUTO_LT, 0 );
		WriteCOMPOOL_IS( SCP_FWD_DAP_MAN_INRTL_LT, 1 );
		WriteCOMPOOL_IS( SCP_AFT_DAP_MAN_INRTL_LT, 1 );
		WriteCOMPOOL_IS( SCP_FWD_DAP_MAN_LVLH_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_DAP_MAN_LVLH_LT, 0 );
		WriteCOMPOOL_IS( SCP_FWD_DAP_MAN_FREE_DFT_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_DAP_MAN_FREE_DFT_LT, 0 );
	}
	else if (DAPControlMode == LVLH)
	{
		WriteCOMPOOL_IS( SCP_FWD_DAP_AUTO_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_DAP_AUTO_LT, 0 );
		WriteCOMPOOL_IS( SCP_FWD_DAP_MAN_INRTL_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_DAP_MAN_INRTL_LT, 0 );
		WriteCOMPOOL_IS( SCP_FWD_DAP_MAN_LVLH_LT, 1 );
		WriteCOMPOOL_IS( SCP_AFT_DAP_MAN_LVLH_LT, 1 );
		WriteCOMPOOL_IS( SCP_FWD_DAP_MAN_FREE_DFT_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_DAP_MAN_FREE_DFT_LT, 0 );
	}
	else// if (DAPControlMode == FREE)
	{
		WriteCOMPOOL_IS( SCP_FWD_DAP_AUTO_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_DAP_AUTO_LT, 0 );
		WriteCOMPOOL_IS( SCP_FWD_DAP_MAN_INRTL_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_DAP_MAN_INRTL_LT, 0 );
		WriteCOMPOOL_IS( SCP_FWD_DAP_MAN_LVLH_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_DAP_MAN_LVLH_LT, 0 );
		WriteCOMPOOL_IS( SCP_FWD_DAP_MAN_FREE_DFT_LT, 1 );
		WriteCOMPOOL_IS( SCP_AFT_DAP_MAN_FREE_DFT_LT, 1 );
	}

	if (PCTActive)
	{
		WriteCOMPOOL_IS( SCP_FWD_BLANK_LT, 1 );
		WriteCOMPOOL_IS( SCP_AFT_BLANK_LT, 1 );
	}
	else
	{
		WriteCOMPOOL_IS( SCP_FWD_BLANK_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_BLANK_LT, 0 );
	}

	// TODO LOW Z
	WriteCOMPOOL_IS( SCP_FWD_TRANSL_LOW_Z_LT, 0 );
	WriteCOMPOOL_IS( SCP_AFT_TRANSL_LOW_Z_LT, 0 );

	// TODO HIGH Z
	WriteCOMPOOL_IS( SCP_FWD_TRANSL_Z_HIGH_LT, 0 );
	WriteCOMPOOL_IS( SCP_AFT_TRANSL_Z_HIGH_LT, 0 );

	if (DAPMode == PRI)
	{
		WriteCOMPOOL_IS( SCP_FWD_RCS_JETS_NORM_LT, 1 );
		WriteCOMPOOL_IS( SCP_AFT_RCS_JETS_NORM_LT, 1 );
		WriteCOMPOOL_IS( SCP_FWD_ALT_PRCS_MODE_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_ALT_PRCS_MODE_LT, 0 );
		WriteCOMPOOL_IS( SCP_FWD_RCS_JETS_VERN_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_RCS_JETS_VERN_LT, 0 );
	}
	else if (DAPMode == ALT)
	{
		WriteCOMPOOL_IS( SCP_FWD_RCS_JETS_NORM_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_RCS_JETS_NORM_LT, 0 );
		WriteCOMPOOL_IS( SCP_FWD_ALT_PRCS_MODE_LT, 1 );
		WriteCOMPOOL_IS( SCP_AFT_ALT_PRCS_MODE_LT, 1 );
		WriteCOMPOOL_IS( SCP_FWD_RCS_JETS_VERN_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_RCS_JETS_VERN_LT, 0 );
	}
	else// if (DAPMode == VERN)
	{
		WriteCOMPOOL_IS( SCP_FWD_RCS_JETS_NORM_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_RCS_JETS_NORM_LT, 0 );
		WriteCOMPOOL_IS( SCP_FWD_ALT_PRCS_MODE_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_ALT_PRCS_MODE_LT, 0 );
		WriteCOMPOOL_IS( SCP_FWD_RCS_JETS_VERN_LT, 1 );
		WriteCOMPOOL_IS( SCP_AFT_RCS_JETS_VERN_LT, 1 );
	}

	if (TransMode[0] == NORM)
	{
		WriteCOMPOOL_IS( SCP_FWD_TRANSL_X_NORM_LT, 1 );
		WriteCOMPOOL_IS( SCP_AFT_TRANSL_X_NORM_LT, 1 );
		WriteCOMPOOL_IS( SCP_FWD_TRANSL_X_PULSE_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_TRANSL_X_PULSE_LT, 0 );
	}
	else// if (TransMode[0] == TRANS_PULSE)
	{
		WriteCOMPOOL_IS( SCP_FWD_TRANSL_X_NORM_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_TRANSL_X_NORM_LT, 0 );
		WriteCOMPOOL_IS( SCP_FWD_TRANSL_X_PULSE_LT, 1 );
		WriteCOMPOOL_IS( SCP_AFT_TRANSL_X_PULSE_LT, 1 );
	}

	if (TransMode[1] == NORM)
	{
		WriteCOMPOOL_IS( SCP_FWD_TRANSL_Y_NORM_LT, 1 );
		WriteCOMPOOL_IS( SCP_AFT_TRANSL_Y_NORM_LT, 1 );
		WriteCOMPOOL_IS( SCP_FWD_TRANSL_Y_PULSE_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_TRANSL_Y_PULSE_LT, 0 );
	}
	else// if (TransMode[1] == TRANS_PULSE)
	{
		WriteCOMPOOL_IS( SCP_FWD_TRANSL_Y_NORM_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_TRANSL_Y_NORM_LT, 0 );
		WriteCOMPOOL_IS( SCP_FWD_TRANSL_Y_PULSE_LT, 1 );
		WriteCOMPOOL_IS( SCP_AFT_TRANSL_Y_PULSE_LT, 1 );
	}

	if (TransMode[2] == NORM)
	{
		WriteCOMPOOL_IS( SCP_FWD_TRANSL_Z_NORM_LT, 1 );
		WriteCOMPOOL_IS( SCP_AFT_TRANSL_Z_NORM_LT, 1 );
		WriteCOMPOOL_IS( SCP_FWD_TRANSL_Z_PULSE_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_TRANSL_Z_PULSE_LT, 0 );
	}
	else// if (TransMode[2] == TRANS_PULSE)
	{
		WriteCOMPOOL_IS( SCP_FWD_TRANSL_Z_NORM_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_TRANSL_Z_NORM_LT, 0 );
		WriteCOMPOOL_IS( SCP_FWD_TRANSL_Z_PULSE_LT, 1 );
		WriteCOMPOOL_IS( SCP_AFT_TRANSL_Z_PULSE_LT, 1 );
	}

	if (RotMode[ROLL] == DISC_RATE)
	{
		WriteCOMPOOL_IS( SCP_FWD_ROT_ROLL_DISC_LT, 1 );
		WriteCOMPOOL_IS( SCP_AFT_ROT_ROLL_DISC_LT, 1 );
		WriteCOMPOOL_IS( SCP_FWD_ROT_ROLL_PULSE_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_ROT_ROLL_PULSE_LT, 0 );
	}
	else// if (RotMode[ROLL] == ROT_PULSE)
	{
		WriteCOMPOOL_IS( SCP_FWD_ROT_ROLL_DISC_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_ROT_ROLL_DISC_LT, 0 );
		WriteCOMPOOL_IS( SCP_FWD_ROT_ROLL_PULSE_LT, 1 );
		WriteCOMPOOL_IS( SCP_AFT_ROT_ROLL_PULSE_LT, 1 );
	}

	if (RotMode[PITCH] == DISC_RATE)
	{
		WriteCOMPOOL_IS( SCP_FWD_ROT_PITCH_DISC_LT, 1 );
		WriteCOMPOOL_IS( SCP_AFT_ROT_PITCH_DISC_LT, 1 );
		WriteCOMPOOL_IS( SCP_FWD_ROT_PITCH_PULSE_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_ROT_PITCH_PULSE_LT, 0 );
	}
	else// if (RotMode[PITCH] == ROT_PULSE)
	{
		WriteCOMPOOL_IS( SCP_FWD_ROT_PITCH_DISC_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_ROT_PITCH_DISC_LT, 0 );
		WriteCOMPOOL_IS( SCP_FWD_ROT_PITCH_PULSE_LT, 1 );
		WriteCOMPOOL_IS( SCP_AFT_ROT_PITCH_PULSE_LT, 1 );
	}

	if (RotMode[YAW] == DISC_RATE)
	{
		WriteCOMPOOL_IS( SCP_FWD_ROT_YAW_DISC_LT, 1 );
		WriteCOMPOOL_IS( SCP_AFT_ROT_YAW_DISC_LT, 1 );
		WriteCOMPOOL_IS( SCP_FWD_ROT_YAW_PULSE_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_ROT_YAW_PULSE_LT, 0 );
	}
	else// if (RotMode[YAW] == ROT_PULSE)
	{
		WriteCOMPOOL_IS( SCP_FWD_ROT_YAW_DISC_LT, 0 );
		WriteCOMPOOL_IS( SCP_AFT_ROT_YAW_DISC_LT, 0 );
		WriteCOMPOOL_IS( SCP_FWD_ROT_YAW_PULSE_LT, 1 );
		WriteCOMPOOL_IS( SCP_AFT_ROT_YAW_PULSE_LT, 1 );
	}

	return;
}

void OrbitDAP::DAP_PBI_Press( void )
{
	// A
	unsigned short FWD_DAP_SELECT_A = ReadCOMPOOL_IS( SCP_FWD_DAP_SELECT_A );
	unsigned short AFT_DAP_SELECT_A = ReadCOMPOOL_IS( SCP_AFT_DAP_SELECT_A );
	if ((FWD_DAP_SELECT_A == 1) || (AFT_DAP_SELECT_A == 1))
	{
		if (DAPSelect != A)
		{
			DAPSelect = A;
			UpdateDAPParameters();
		}
	}

	// B
	unsigned short FWD_DAP_SELECT_B = ReadCOMPOOL_IS( SCP_FWD_DAP_SELECT_B );
	unsigned short AFT_DAP_SELECT_B = ReadCOMPOOL_IS( SCP_AFT_DAP_SELECT_B );
	if ((FWD_DAP_SELECT_B == 1) || (AFT_DAP_SELECT_B == 1))
	{
		if (DAPSelect != B)
		{
			DAPSelect = B;
			UpdateDAPParameters();
		}
	}

	// AUTO
	unsigned short FWD_DAP_AUTO = ReadCOMPOOL_IS( SCP_FWD_DAP_AUTO );
	unsigned short AFT_DAP_AUTO = ReadCOMPOOL_IS( SCP_AFT_DAP_AUTO );
	if ((FWD_DAP_AUTO == 1) || (AFT_DAP_AUTO == 1))
	{
		if (DAPControlMode != AUTO)
		{
			DAPControlMode = AUTO;
			if(CurManeuver.IsValid) StartCurManeuver();
		}
	}

	// INRTL
	unsigned short FWD_DAP_INRTL = ReadCOMPOOL_IS( SCP_FWD_DAP_MAN );
	unsigned short AFT_DAP_INRTL = ReadCOMPOOL_IS( SCP_AFT_DAP_MAN );
	if ((FWD_DAP_INRTL == 1) || (AFT_DAP_INRTL == 1))
	{
		if (DAPControlMode != INRTL)
		{
			DAPControlMode = INRTL;
			StartManeuver(curM50Matrix, AttManeuver::MNVR);
		}
	}

	// LVLH
	unsigned short FWD_DAP_LVLH = ReadCOMPOOL_IS( SCP_FWD_RCS_OPTIONS_NORM );
	unsigned short AFT_DAP_LVLH = ReadCOMPOOL_IS( SCP_AFT_RCS_OPTIONS_NORM );
	if ((FWD_DAP_LVLH == 1) || (AFT_DAP_LVLH == 1))
	{
		if (DAPControlMode != LVLH)
		{
			DAPControlMode = LVLH;
			StartManeuver(GetCurrentLVLHAttMatrix(), AttManeuver::LVLH_TRK);
		}
	}

	// FREE
	unsigned short FWD_DAP_FREE = ReadCOMPOOL_IS( SCP_FWD_RCS_OPTIONS_VERNIER );
	unsigned short AFT_DAP_FREE = ReadCOMPOOL_IS( SCP_AFT_RCS_OPTIONS_VERNIER );
	if ((FWD_DAP_FREE == 1) || (AFT_DAP_FREE == 1))
	{
		if (DAPControlMode != FREE)
		{
			DAPControlMode = FREE;
			for(unsigned int i=0;i<3;i++) {
				RotMode[i]=ROT_PULSE;
				RotPulseInProg[i]=false;
			}
		}
	}

	// (blank)
	// LOW Z
	// HIGH Z

	// PRI
	unsigned short FWD_RCS_PRI = ReadCOMPOOL_IS( SCP_FWD_ROTATION_ROLL_DISC_RATE );
	unsigned short AFT_RCS_PRI = ReadCOMPOOL_IS( SCP_AFT_ROTATION_ROLL_DISC_RATE );
	if ((FWD_RCS_PRI == 1) || (AFT_RCS_PRI == 1))
	{
		if (DAPMode != PRI)
		{
			DAPMode = PRI;
			UpdateDAPParameters();
		}
	}

	// ALT
	unsigned short FWD_RCS_ALT = ReadCOMPOOL_IS( SCP_FWD_ROTATION_PITCH_DISC_RATE );
	unsigned short AFT_RCS_ALT = ReadCOMPOOL_IS( SCP_AFT_ROTATION_PITCH_DISC_RATE );
	if ((FWD_RCS_ALT == 1) || (AFT_RCS_ALT == 1))
	{
		if (DAPMode != ALT)
		{
			DAPMode = ALT;
			UpdateDAPParameters();
		}
	}

	// VERN
	unsigned short FWD_RCS_VERN = ReadCOMPOOL_IS( SCP_FWD_ROTATION_YAW_DISC_RATE );
	unsigned short AFT_RCS_VERN = ReadCOMPOOL_IS( SCP_AFT_ROTATION_YAW_DISC_RATE );
	if ((FWD_RCS_VERN == 1) || (AFT_RCS_VERN == 1))
	{
		if (DAPMode != VERN)
		{
			DAPMode = VERN;
			UpdateDAPParameters();
		}
	}

	// TRANS X NORM
	unsigned short FWD_TRANSLATION_X_NORM = ReadCOMPOOL_IS( SCP_FWD_TRANSLATION_X_NORM );
	unsigned short AFT_TRANSLATION_X_NORM = ReadCOMPOOL_IS( SCP_AFT_TRANSLATION_X_NORM );
	if ((FWD_TRANSLATION_X_NORM == 1) || (AFT_TRANSLATION_X_NORM == 1))
	{
		if (TransMode[0] != NORM)
		{
			TransMode[0]=NORM;
			TransPulseInProg[0]=false;
			TransPulseDV.x=0.0;
		}
	}

	// TRANS Y NORM
	unsigned short FWD_TRANSLATION_Y_NORM = ReadCOMPOOL_IS( SCP_FWD_TRANSLATION_Y_NORM );
	unsigned short AFT_TRANSLATION_Y_NORM = ReadCOMPOOL_IS( SCP_AFT_TRANSLATION_Y_NORM );
	if ((FWD_TRANSLATION_Y_NORM == 1) || (AFT_TRANSLATION_Y_NORM == 1))
	{
		if (TransMode[1] != NORM)
		{
			TransMode[1]=NORM;
			TransPulseInProg[1]=false;
			TransPulseDV.y=0.0;
		}
	}

	// TRANS Z NORM
	unsigned short FWD_TRANSLATION_Z_NORM = ReadCOMPOOL_IS( SCP_FWD_TRANSLATION_Z_NORM );
	unsigned short AFT_TRANSLATION_Z_NORM = ReadCOMPOOL_IS( SCP_AFT_TRANSLATION_Z_NORM );
	if ((FWD_TRANSLATION_Z_NORM == 1) || (AFT_TRANSLATION_Z_NORM == 1))
	{
		if (TransMode[2] != NORM)
		{
			TransMode[2]=NORM;
			TransPulseInProg[2]=false;
			TransPulseDV.z=0.0;
		}
	}

	// ROLL DISC RATE
	unsigned short FWD_ROTATION_ROLL_ACCEL = ReadCOMPOOL_IS( SCP_FWD_ROTATION_ROLL_ACCEL );
	unsigned short AFT_ROTATION_ROLL_ACCEL = ReadCOMPOOL_IS( SCP_AFT_ROTATION_ROLL_ACCEL );
	if ((FWD_ROTATION_ROLL_ACCEL == 1) || (AFT_ROTATION_ROLL_ACCEL == 1))
	{
		if (RotMode[ROLL] != DISC_RATE)
		{
			RotMode[ROLL]=DISC_RATE;
			RotPulseInProg[ROLL]=false;
		}
	}

	// PITCH DISC RATE
	unsigned short FWD_ROTATION_PITCH_ACCEL = ReadCOMPOOL_IS( SCP_FWD_ROTATION_PITCH_ACCEL );
	unsigned short AFT_ROTATION_PITCH_ACCEL = ReadCOMPOOL_IS( SCP_AFT_ROTATION_PITCH_ACCEL );
	if ((FWD_ROTATION_PITCH_ACCEL == 1) || (AFT_ROTATION_PITCH_ACCEL == 1))
	{
		if (RotMode[PITCH] != DISC_RATE)
		{
			RotMode[PITCH]=DISC_RATE;
			RotPulseInProg[PITCH]=false;
		}
	}

	// YAW DISC RATE
	unsigned short FWD_ROTATION_YAW_ACCEL = ReadCOMPOOL_IS( SCP_FWD_ROTATION_YAW_ACCEL );
	unsigned short AFT_ROTATION_YAW_ACCEL = ReadCOMPOOL_IS( SCP_AFT_ROTATION_YAW_ACCEL );
	if ((FWD_ROTATION_YAW_ACCEL == 1) || (AFT_ROTATION_YAW_ACCEL == 1))
	{
		if (RotMode[YAW] != DISC_RATE)
		{
			RotMode[YAW]=DISC_RATE;
			RotPulseInProg[YAW]=false;
		}
	}

	// TRANS X PULSE
	unsigned short FWD_TRANSLATION_X_PULSE = ReadCOMPOOL_IS( SCP_FWD_TRANSLATION_X_PULSE );
	unsigned short AFT_TRANSLATION_X_PULSE = ReadCOMPOOL_IS( SCP_AFT_TRANSLATION_X_PULSE );
	if ((FWD_TRANSLATION_X_PULSE == 1) || (AFT_TRANSLATION_X_PULSE == 1))
	{
		if ((TransMode[0] != TRANS_PULSE) && ((GetMajorMode() == 201) || (GetMajorMode() == 202)))
		{
			TransMode[0]=TRANS_PULSE;
			TransPulseInProg[0]=false;
			TransPulseDV.x=0.0;
		}
	}

	// TRANS Y PULSE
	unsigned short FWD_TRANSLATION_Y_PULSE = ReadCOMPOOL_IS( SCP_FWD_TRANSLATION_Y_PULSE );
	unsigned short AFT_TRANSLATION_Y_PULSE = ReadCOMPOOL_IS( SCP_AFT_TRANSLATION_Y_PULSE );
	if ((FWD_TRANSLATION_Y_PULSE == 1) || (AFT_TRANSLATION_Y_PULSE == 1))
	{
		if ((TransMode[1] != TRANS_PULSE) && ((GetMajorMode() == 201) || (GetMajorMode() == 202)))
		{
			TransMode[1]=TRANS_PULSE;
			TransPulseInProg[1]=false;
			TransPulseDV.y=0.0;
		}
	}

	// TRANS Z PULSE
	unsigned short FWD_TRANSLATION_Z_PULSE = ReadCOMPOOL_IS( SCP_FWD_TRANSLATION_Z_PULSE );
	unsigned short AFT_TRANSLATION_Z_PULSE = ReadCOMPOOL_IS( SCP_AFT_TRANSLATION_Z_PULSE );
	if ((FWD_TRANSLATION_Z_PULSE == 1) || (AFT_TRANSLATION_Z_PULSE == 1))
	{
		if ((TransMode[2] != TRANS_PULSE) && ((GetMajorMode() == 201) || (GetMajorMode() == 202)))
		{
			TransMode[2]=TRANS_PULSE;
			TransPulseInProg[2]=false;
			TransPulseDV.z=0.0;
		}
	}

	// ROLL PULSE
	unsigned short FWD_ROTATION_ROLL_PULSE = ReadCOMPOOL_IS( SCP_FWD_ROTATION_ROLL_PULSE );
	unsigned short AFT_ROTATION_ROLL_PULSE = ReadCOMPOOL_IS( SCP_AFT_ROTATION_ROLL_PULSE );
	if ((FWD_ROTATION_ROLL_PULSE == 1) || (AFT_ROTATION_ROLL_PULSE == 1))
	{
		if (RotMode[ROLL] != ROT_PULSE)
		{
			RotMode[ROLL]=ROT_PULSE;
			RotPulseInProg[ROLL] = false;
		}
	}

	// PITCH PULSE
	unsigned short FWD_ROTATION_PITCH_PULSE = ReadCOMPOOL_IS( SCP_FWD_ROTATION_PITCH_PULSE );
	unsigned short AFT_ROTATION_PITCH_PULSE = ReadCOMPOOL_IS( SCP_AFT_ROTATION_PITCH_PULSE );
	if ((FWD_ROTATION_PITCH_PULSE == 1) || (AFT_ROTATION_PITCH_PULSE == 1))
	{
		if (RotMode[PITCH] != ROT_PULSE)
		{
			RotMode[PITCH]=ROT_PULSE;
			RotPulseInProg[PITCH] = false;
		}
	}

	// YAW PULSE
	unsigned short FWD_ROTATION_YAW_PULSE = ReadCOMPOOL_IS( SCP_FWD_ROTATION_YAW_PULSE );
	unsigned short AFT_ROTATION_YAW_PULSE = ReadCOMPOOL_IS( SCP_AFT_ROTATION_YAW_PULSE );
	if ((FWD_ROTATION_YAW_PULSE == 1) || (AFT_ROTATION_YAW_PULSE == 1))
	{
		if (RotMode[YAW] != ROT_PULSE)
		{
			RotMode[YAW]=ROT_PULSE;
			RotPulseInProg[YAW] = false;
		}
	}
	return;
}

void OrbitDAP::ArmPCT( void )
{
	PCTArmed = true;
	return;
}

void OrbitDAP::DisarmPCT( void )
{
	PCTArmed = false;
	return;
}

void OrbitDAP::StartPCT()
{
	PCTActive = true;
	PCTStartTime = oapiGetSimTime();
	DAPMode = PRI;
	DAPControlMode = FREE;
	return;
}

void OrbitDAP::StopPCT()
{
	PCTActive = false;
	DAPSelect = A;
	DAPMode = VERN;
	// TODO: mode to A9/B9 DAP CONFIGs

	// TODO stop firing RCS jets
	return;
}

void OrbitDAP::PCTControl(double simt)
{
	double dT=simt-PCTStartTime;

	//prevent translation thrusters except (up group) from firing
	TransThrusterCommands[0].ResetLine();
	TransThrusterCommands[1].ResetLine();

	//fire thrusters as appropriate
	if(dT<=PCT_STAGE1) TransThrusterCommands[2].SetLine(-0.5f);
	else if(dT<=PCT_STAGE2) TransThrusterCommands[2].ResetLine();
	else if(dT<=PCT_STAGE3) TransThrusterCommands[2].SetLine(-0.5f);
	else {
		TransThrusterCommands[2].ResetLine();
		StopPCT();
	}
}

void OrbitDAP::UpdateDAPParameters()
{
	int profile;
	if (DAPSelect == 0)
		profile = activeAProfile;
	else if (DAPSelect == 1)
		profile = activeBProfile;
	else
		profile = 0;

	TransPulse=DAPConfiguration[DAPSelect][profile].PRI_TRAN_PLS * FPS2MS; //same for all modes
	if(DAPMode==PRI) {
		degRotRate=DAPConfiguration[DAPSelect][profile].PRI_ROT_RATE;
		degAttDeadband=DAPConfiguration[DAPSelect][profile].PRI_ATT_DB;
		degRateDeadband=DAPConfiguration[DAPSelect][profile].PRI_RATE_DB;
		degRotPulse=DAPConfiguration[DAPSelect][profile].PRI_ROT_PLS;

		if(DAPConfiguration[DAPSelect][profile].PRI_P_OPTION!=0)
			Torque.data[PITCH]=0.5*ORBITER_PITCH_TORQUE;
		else Torque.data[PITCH]=ORBITER_PITCH_TORQUE;
		if(DAPConfiguration[DAPSelect][profile].PRI_Y_OPTION!=0)
			Torque.data[YAW]=0.5*ORBITER_YAW_TORQUE;
		else Torque.data[YAW]=ORBITER_YAW_TORQUE;
		Torque.data[ROLL]=ORBITER_ROLL_TORQUE;
	}
	else if(DAPMode==ALT) {
		degRotRate=DAPConfiguration[DAPSelect][profile].PRI_ROT_RATE;
		degAttDeadband=DAPConfiguration[DAPSelect][profile].PRI_ATT_DB;
		degRateDeadband=DAPConfiguration[DAPSelect][profile].ALT_RATE_DB;
		degRotPulse=DAPConfiguration[DAPSelect][profile].PRI_ROT_PLS;
		if(DAPConfiguration[DAPSelect][profile].ALT_JET_OPT==2) {
			Torque.data[PITCH]=0.5*ORBITER_PITCH_TORQUE;
			Torque.data[YAW]=0.5*ORBITER_YAW_TORQUE;
			Torque.data[ROLL]=0.5*ORBITER_ROLL_TORQUE;
		}
		else {
			Torque.data[PITCH]=ORBITER_PITCH_TORQUE;
			Torque.data[YAW]=ORBITER_YAW_TORQUE;
			Torque.data[ROLL]=ORBITER_ROLL_TORQUE;
		}
	}
	else if(DAPMode==VERN) {
		degRotRate=DAPConfiguration[DAPSelect][profile].VERN_ROT_RATE;
		degAttDeadband=DAPConfiguration[DAPSelect][profile].VERN_ATT_DB;
		degRateDeadband=DAPConfiguration[DAPSelect][profile].VERN_RATE_DB;
		degRotPulse=DAPConfiguration[DAPSelect][profile].VERN_ROT_PLS;
		Torque.data[PITCH]=0.1*ORBITER_PITCH_TORQUE;
		Torque.data[YAW]=0.1*ORBITER_YAW_TORQUE;
		Torque.data[ROLL]=0.1*ORBITER_ROLL_TORQUE;
	}
}

double OrbitDAP::CalcManeuverCompletionTime(const MATRIX3& curM50Matrix, const MATRIX3& tgtLVLHMatrix, const MATRIX3& curLVLHMatrix, double degOrbitalRate) const
{
	double mnvrTime = 0.0;
	double lastMnvrTime = 0.0;
	int counter = 0;
	VECTOR3 Axis;
	MATRIX3 PYR;
	do {
		counter++;
		lastMnvrTime=mnvrTime;
		if(mnvrTime < 0) {
			mnvrTime=0.0;
		}

		double rotationAngle = degOrbitalRate*mnvrTime;
		MATRIX3 LVLHRotation;
		GetRotMatrixY(-rotationAngle*RAD, LVLHRotation);
		MATRIX3 tgtM50Matrix = mul(mul(curLVLHMatrix, LVLHRotation), tgtLVLHMatrix);

		PYR = GetRotationErrorMatrix(curM50Matrix, tgtM50Matrix);
		double Angle=CalcEulerAngle(PYR, Axis);
		mnvrTime=(Angle*DEG)/degRotRate;
	} while(abs(mnvrTime-lastMnvrTime)>0.05 && counter<50);
	return max(mnvrTime, 0.0);
}


MATRIX3 OrbitDAP::GetCurrentLVLHRefMatrix() const
{
	VECTOR3 pos, vel;
	pStateVector->GetCurrentStateVectorsM50(pos, vel);
	return Transpose(GetGlobalToLVLHMatrix(pos, vel));
}

MATRIX3 OrbitDAP::GetCurrentLVLHAttMatrix() const
{
	return GetRotationErrorMatrix(GetCurrentLVLHRefMatrix(), curM50Matrix);
}

int OrbitDAP::GetCurrentProfile(int item)
{
	switch (convert[item])
	{
	case 0:
		return activeAProfile;
	case 1:
		return activeBProfile;
	default:
		return 0;
	}
}

VECTOR3 OrbitDAP::GetAttitudeErrors( void ) const
{
	return -ATT_ERR;
}

bool OrbitDAP::GetTimeToAttitude( double& time ) const
{
	if (ManeuverStatus != MNVR_IN_PROGRESS) return false;
	time = max( mnvrCompletionMET - STS()->GetMET(), 0 );
	return true;
}

}
