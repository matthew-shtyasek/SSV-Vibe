#include "UniversalPointing.h"
#include "../../IDP.h"
#include "GNCUtilities.h"
#include "../../../vc/MDU.h"
#include "OrbitDAP.h"
#include "StarCatalog.h"

namespace dps
{
	const float ATT_TOLERANCE = 1.1f; //Maneuver completion time freeze tolerance

	const double Q_M50_INRTL_ORB_S = 1.0;
	const VECTOR3 Q_M50_INRTL_ORB_V = _V(0, 0, 0);
	const float RNG_TOL_UPTG = 100.0f; //Inertial rate range tolerance
	const float RTE_LIM = 3.0f; //Tracking rate limit
	const float TLOS_TOL = 0.999848f; //Alternate target vector tolerance
	const float PLOS_TOL = 0.999848f; //Alternate body vector tolerance

	UniversalPointing::UniversalPointing(SimpleGPCSystem* _gpc) : SimpleGPCSoftware(_gpc, "UniversalPointing"),
		pOrbitDAP(NULL), pGNCUtilities(NULL), lastUpdateSimTime(-100.0)
	{
		OPT_SEL = 0;
		OPT_CNCL = false;
		TRK_DCD = ROT_DCD = MNV_DCD = FUT_DCD = false;
		PARAM_XFR = false;
		MNVR_CMPL_TIME = 0.0;
		GMTS = 0.0;
		ROLLC = PITCHC = YAWC = 0.0f;
		ROLLR = PITCHR = YAWR = 0.0f;
		ATT_ERR = _V(0, 0, 0);
		ATT_RATE = _V(0, 0, 0);

		GMT = 0.0;
		Q_RBOD_M50_S = 0.0;
		Q_RBOD_M50_V = _V(0, 0, 0);
		REQD_BRATE = _V(0, 0, 0);
		GMTR = 0.0;
		MNVR_TRACK_OPTION = false;

		PLOS = _V(0, 0, 0);
		TGPOS = _V(0, 0, 0);

		PLOS_C = _V(0, 0, 0);
		TGT_ID_C = 0;
		TGPOS_C = _V(0, 0, 0);
		OMICRON_C = 0.0;
		THREE_AXIS_C = false;
		ROTR_RATE_C = 0.0;
		GMTS_C = 0.0;

		Q_MNVR_S = Q_BODC_M50_S = Q_MNVRC_S = 0.0;
		Q_MNVR_V = Q_BODC_M50_V = Q_MNVRC_V = _V(0, 0, 0);

		MNVR_TIME_IC = MNVR_TIME_FLAG = false;

		RA_flash = DEC_flash = LAT_flash = LON_flash = ALT_flash = P_flash = Y_flash = OMICRON_flash = false;
		PY_executed = RA_DEC_executed = false;
		AUTO_ALERT = false;

		BODV_SCL[0][0] = 0.0f; BODV_SCL[0][1] = 0.0f;
		BODV_SCL[1][0] = 3.14159f; BODV_SCL[1][1] = 0.0f;
		BODV_SCL[2][0] = 1.57079f; BODV_SCL[2][1] = 0.0f;
		BODV_SCL[3][0] = 0.0f; BODV_SCL[3][1] = 4.89687f;

		KLoad();
	}

	UniversalPointing::~UniversalPointing()
	{

	}

	void UniversalPointing::Realize()
	{
		pGNCUtilities = dynamic_cast<GNCUtilities*>(FindSoftware("GNCUtilities"));
		assert((pGNCUtilities != NULL) && "OMSBurnSoftware::Realize.pGNCUtilities");
		pOrbitDAP = dynamic_cast<OrbitDAP*>(FindSoftware("OrbitDAP"));
		assert((pOrbitDAP != NULL) && "OMSBurnSoftware::Realize.pOrbitDAP");
	}

	void UniversalPointing::OnPreStep(double simt, double simdt, double mjd)
	{
		if ((simt - lastUpdateSimTime) > 1.92)
		{
			// Read
			GMT = ReadClock();
			GMTC = GMT; //TBD: Should be time tag of attitude quaternion from attitude processor
			Q_BOD_M50_S = ReadCOMPOOL_VS(SCP_Q_B_I, 1, 4);
			Q_BOD_M50_V = ReadCOMPOOL_VS(SCP_Q_B_I + 2);
			QUAT_NORM(Q_BOD_M50_S, Q_BOD_M50_V); //TBD: Shouldn't be necessary. Issue with double/float conversions?
			RORB = ReadCOMPOOL_VD(SCP_R_AVGG);
			VORB = ReadCOMPOOL_VD(SCP_V_AVGG);
			RREL = ReadCOMPOOL_VD(SCP_DEL_R_TARG);
			VREL = ReadCOMPOOL_VD(SCP_DEL_V_TARG);

			// Perform Universal Pointing Processing Sequencing Task
			UNIV_SEQ();

			// Write for GAX
			if (AUTO_ALERT)
			{
				WriteCOMPOOL_IS(SCP_SEL_AUTO_CREW_ALERT, 1);
			}
			else
			{
				WriteCOMPOOL_IS(SCP_SEL_AUTO_CREW_ALERT, 0);
			}

			lastUpdateSimTime = simt;
		}
	}

	bool UniversalPointing::OnMajorModeChange(unsigned int newMajorMode)
	{
		if (newMajorMode == 201 || newMajorMode == 202 || newMajorMode == 801)
		{
			int ops = GetMajorMode() / 100;

			//Protect memory for transitions within OPS 2 and 8. And also during scenario load (MajorMode == 0)
			if (ops != 0 && ops != 2 && ops != 8)
			{
				//Initialize
				Q_RBOD_M50_S = ReadCOMPOOL_VS(SCP_Q_B_I, 1, 4);
				Q_RBOD_M50_V = ReadCOMPOOL_VS(SCP_Q_B_I + 2);

				KLoad();
			}
			return true;
		}
		return false;
	}

	bool UniversalPointing::ItemInput(int item, const char* Data)
	{
		switch (item)
		{
		case 1:
		{
			int nNew = 0;
			if (GetIntegerUnsigned(Data, nNew))
			{
				if (nNew < 365)
				{
					START_TIME[0] = nNew;
					FUT_DCD = true;
				}
				else return false;
			}
			else return false;
		}
		break;
		case 2:
		{
			int nNew = 0;
			if (GetIntegerUnsigned(Data, nNew))
			{
				if (nNew < 24)
				{
					START_TIME[1] = nNew;
					FUT_DCD = true;
				}
				else return false;
			}
			else return false;
		}
		break;
		case 3:
		{
			int nNew = 0;
			if (GetIntegerUnsigned(Data, nNew))
			{
				if (nNew < 60)
				{
					START_TIME[2] = nNew;
					FUT_DCD = true;
				}
				else return false;
			}
			else return false;
		}
		break;
		case 4:
		{
			int nNew = 0;
			if (GetIntegerUnsigned(Data, nNew))
			{
				if (nNew < 60)
				{
					START_TIME[3] = nNew;
					FUT_DCD = true;
				}
				else return false;
			}
			else return false;
		}
		break;
		case 5:
		{
			double dNew;
			if (GetDoubleUnsigned(Data, dNew))
			{
				if (dNew < 359.99)
				{
					ROLLI = RAD_PER_DEG * (float)(dNew);
					MNV_DCD = true;
				}
				else return false;
			}
			else return false;
		}
		break;
		case 6:
		{
			double dNew;
			if (GetDoubleUnsigned(Data, dNew))
			{
				if (dNew < 359.99)
				{
					PITCHI = RAD_PER_DEG * (float)(dNew);
					MNV_DCD = true;
				}
				else return false;
			}
			else return false;
		}
		break;
		case 7:
		{
			double dNew;
			if (GetDoubleUnsigned(Data, dNew))
			{
				if ((dNew <= 90.0) || ((dNew >= 270.0) && (dNew < 359.99)))
				{
					YAWI = RAD_PER_DEG * (float)(dNew);
					MNV_DCD = true;
				}
				else return false;
			}
			else return false;
		}
		break;
		case 8:
		{
			int nNew;
			if (GetIntegerUnsigned(Data, nNew))
			{
				if ((nNew >= 1 && nNew <= 5) || (nNew >= 11 && nNew <= 110))
				{
					TGT_ID = nNew;
					TRK_DCD = true;
					THREE_AXIS = false;

					if (TGT_ID == 3)
					{
						LAT_flash = LON_flash = ALT_flash = true;
					}
					else
					{
						LAT_flash = LON_flash = ALT_flash = false;
					}

					if (TGT_ID == 5)
					{
						RA_flash = DEC_flash = true;
					}
					else
					{
						RA_flash = DEC_flash = false;
					}
				}
				else return false;
			}
			else return false;
		}
		break;
		case 9:
		{
			double dNew;
			if (GetDoubleUnsigned(Data, dNew))
			{
				if ((dNew <= 359.999) && (TGT_ID == 5))
				{
					TGT_RA = RAD_PER_DEG * (float)(dNew);
					TRK_DCD = true;
					RA_flash = false;
					RA_DEC_executed = true;
				}
				else return false;
			}
			else return false;
		}
		break;
		case 10:
		{
			double dNew;
			if (GetDoubleSigned(Data, dNew))
			{
				if ((dNew >= -90.0) && (dNew <= 90.0) && (TGT_ID == 5))
				{
					TGT_DEC = RAD_PER_DEG * (float)(dNew);
					TRK_DCD = true;
					DEC_flash = false;
					RA_DEC_executed = true;
				}
				else return false;
			}
			else return false;
		}
		break;
		case 11:
		{
			double dNew;
			if (GetDoubleSigned(Data, dNew))
			{
				if ((dNew >= -90.0) && (dNew <= 90.0) && (TGT_ID == 3))
				{
					TGT_LAT = RAD_PER_DEG * (float)(dNew);
					TRK_DCD = true;
					LAT_flash = false;
				}
				else return false;
			}
			else return false;
		}
		break;
		case 12:
		{
			double dNew;
			if (GetDoubleSigned(Data, dNew))
			{
				if ((dNew >= -180.0) && (dNew <= 180.0) && (TGT_ID == 3))
				{
					TGT_LON = RAD_PER_DEG * (float)(dNew);
					TRK_DCD = true;
					LON_flash = false;
				}
				else return false;
			}
			else return false;
		}
		break;
		case 13:
		{
			double dNew;
			if (GetDoubleSigned(Data, dNew))
			{
				if ((dNew >= -3444.0) && (dNew <= 20000.0) && (TGT_ID == 3))
				{
					TGT_ALT = (float)(dNew);// [nmi] (SSV: not an angle, the factor of the degrees was a copy-paste error of the original)
					TRK_DCD = true;
					ALT_flash = false;
				}
				else return false;
			}
			else return false;
		}
		break;
		case 14:
		{
			int nNew;
			if (GetIntegerUnsigned(Data, nNew))
			{
				if ((nNew >= 1) && (nNew <= 5))
				{
					BODV_ID = nNew;
					TRK_DCD = true;
					ROT_DCD = true;
					OMICRON_flash = true;

					if (BODV_ID == 5)
					{
						P_flash = Y_flash = true;
					}
					else
					{
						P_flash = Y_flash = false;
					}
				}
				else return false;
			}
			else return false;
		}
		break;
		case 15:
		{
			double dNew;
			if (GetDoubleUnsigned(Data, dNew))
			{
				if ((dNew < 359.99) && (BODV_ID == 5))
				{
					BODV_PITCH = RAD_PER_DEG * (float)(dNew);
					TRK_DCD = true;
					ROT_DCD = true;
					P_flash = false;
					PY_executed = true;
				}
				else return false;
			}
			else return false;
		}
		break;
		case 16:
		{
			double dNew;
			if (GetDoubleUnsigned(Data, dNew))
			{
				if (((dNew <= 90.0) || ((dNew >= 270.0) && (dNew < 359.99))) && (BODV_ID == 5))
				{
					BODV_YAW = RAD_PER_DEG * (float)(dNew);
					TRK_DCD = true;
					ROT_DCD = true;
					Y_flash = false;
					PY_executed = true;
				}
				else return false;
			}
			else return false;
		}
		break;
		case 17:
		{
			double dNew;
			if (GetDoubleUnsigned(Data, dNew))
			{
				if (dNew < 359.99)
				{
					OMICRON = RAD_PER_DEG * (float)(dNew);
					TRK_DCD = true;
					OMICRON_flash = false;
					THREE_AXIS = true;
				}
				else return false;
			}
			else return false;
		}
		break;
		case 18:
		{
			if (strlen(Data) == 0)
			{
				OPT_SEL = 1;
				OPT_CNCL = false;
				OMICRON_flash = false;
				P_flash = Y_flash = false;
				RA_flash = DEC_flash = LAT_flash = LON_flash = ALT_flash = false;
			}
			else return false;
		}
		break;
		case 19:
		{
			if (strlen(Data) == 0)
			{
				OPT_SEL = 2;
				OPT_CNCL = false;
				OMICRON_flash = false;
				P_flash = Y_flash = false;
				RA_flash = DEC_flash = LAT_flash = LON_flash = ALT_flash = false;
			}
			else return false;
		}
		break;
		case 20:
		{
			if (strlen(Data) == 0)
			{
				OPT_SEL = 3;
				OPT_CNCL = false;
				OMICRON_flash = false;
				P_flash = Y_flash = false;
				RA_flash = DEC_flash = LAT_flash = LON_flash = ALT_flash = false;
			}
			else return false;
		}
		break;
		case 21:
		{
			if (strlen(Data) == 0)
			{
				OPT_CNCL = true;
				P_flash = Y_flash = false;
				OMICRON_flash = false;
				RA_flash = DEC_flash = LAT_flash = LON_flash = ALT_flash = false;
			}
			else return false;
		}
		break;
		case 22:
		{
			int nNew;
			if (GetIntegerUnsigned(Data, nNew))
			{
				if ((nNew >= 1) && (nNew <= 2))
				{
					MON_AXIS = nNew;
				}
				else return false;
			}
			else return false;
		}
		break;
		case 23:
			if (strlen(Data) == 0) ERR_SEL = true;
			else return false;
			break;
		case 24:
			if (strlen(Data) == 0) ERR_SEL = false;
			else return false;
			break;
		default:
			return false;
		}
		return true;
	}

	void UniversalPointing::OnPaint(vc::MDU* pMDU) const
	{
		char cbuf[255];
		PrintCommonHeader("    UNIV PTG", pMDU);

		double CUR_MNVR_COMPL[4];
		ConvertSecondsToDDHHMMSS(MNVR_CMPL_TIME, CUR_MNVR_COMPL);
		pMDU->mvprint(3, 1, "CUR MNVR COMPL");
		sprintf_s(cbuf, 255, "%.2d:%.2d:%.2d", static_cast<int>(CUR_MNVR_COMPL[1]), static_cast<int>(CUR_MNVR_COMPL[2]), static_cast<int>(CUR_MNVR_COMPL[3]));
		pMDU->mvprint(18, 1, cbuf);
		sprintf_s(cbuf, 255, "1 START TIME %.3d/%.2d:%.2d:%.2d",
			START_TIME[0], START_TIME[1], START_TIME[2], START_TIME[3]);
		pMDU->mvprint(1, 2, cbuf);
		pMDU->Underline(14, 2);
		pMDU->Underline(15, 2);
		pMDU->Underline(16, 2);
		pMDU->Underline(18, 2);
		pMDU->Underline(19, 2);
		pMDU->Underline(21, 2);
		pMDU->Underline(22, 2);
		pMDU->Underline(24, 2);
		pMDU->Underline(25, 2);

		pMDU->mvprint(0, 4, "MNVR OPTION");
		sprintf_s(cbuf, 255, "5 R %6.2f", ROLLI / RAD_PER_DEG);
		pMDU->mvprint(1, 5, cbuf);
		pMDU->Underline(5, 5);
		pMDU->Underline(6, 5);
		pMDU->Underline(7, 5);
		pMDU->Underline(8, 5);
		pMDU->Underline(9, 5);
		pMDU->Underline(10, 5);
		sprintf_s(cbuf, 255, "6 P %6.2f", PITCHI / RAD_PER_DEG);
		pMDU->mvprint(1, 6, cbuf);
		sprintf_s(cbuf, 255, "7 Y %6.2f", YAWI / RAD_PER_DEG);
		pMDU->mvprint(1, 7, cbuf);

		pMDU->mvprint(0, 9, "TRK/ROT OPTIONS");
		sprintf_s(cbuf, 255, "8 TGT ID %3d", TGT_ID);
		pMDU->mvprint(1, 10, cbuf);
		pMDU->Underline(10, 10);
		pMDU->Underline(11, 10);
		pMDU->Underline(12, 10);

		pMDU->mvprint(1, 12, "9  RA");
		sprintf_s(cbuf, 255, "%7.3f", TGT_RA / RAD_PER_DEG);
		pMDU->mvprint(9, 12, cbuf, RA_flash ? DEUATT_FLASHING : 0);
		pMDU->Underline(9, 12);
		pMDU->Underline(10, 12);
		pMDU->Underline(11, 12);
		pMDU->Underline(12, 12);
		pMDU->Underline(13, 12);
		pMDU->Underline(14, 12);
		pMDU->Underline(15, 12);
		pMDU->mvprint(1, 13, "10 DEC");
		pMDU->NumberSignBracket(9, 13, TGT_DEC);// TODO should brackets flash with sign?
		sprintf_s(cbuf, 255, "%6.3f", fabs(TGT_DEC / RAD_PER_DEG));
		pMDU->mvprint(10, 13, cbuf, DEC_flash ? DEUATT_FLASHING : 0);
		pMDU->Underline(10, 13);
		pMDU->Underline(11, 13);
		pMDU->Underline(12, 13);
		pMDU->Underline(13, 13);
		pMDU->Underline(14, 13);
		pMDU->Underline(15, 13);
		pMDU->mvprint(1, 14, "11 LAT");
		pMDU->NumberSignBracket(9, 14, TGT_LAT);// TODO should brackets flash with sign?
		sprintf_s(cbuf, 255, "%6.3f", fabs(TGT_LAT / RAD_PER_DEG));
		pMDU->mvprint(10, 14, cbuf, LAT_flash ? DEUATT_FLASHING : 0);
		pMDU->Underline(10, 14);
		pMDU->Underline(11, 14);
		pMDU->Underline(12, 14);
		pMDU->Underline(13, 14);
		pMDU->Underline(14, 14);
		pMDU->Underline(15, 14);
		pMDU->mvprint(1, 15, "12 LON");
		pMDU->NumberSignBracket(8, 15, TGT_LON);// TODO should brackets flash with sign?
		sprintf_s(cbuf, 255, "%7.3f", fabs(TGT_LON / RAD_PER_DEG));
		pMDU->mvprint(9, 15, cbuf, LON_flash ? DEUATT_FLASHING : 0);
		pMDU->Underline(9, 15);
		pMDU->Underline(10, 15);
		pMDU->Underline(11, 15);
		pMDU->Underline(12, 15);
		pMDU->Underline(13, 15);
		pMDU->Underline(14, 15);
		pMDU->Underline(15, 15);
		pMDU->mvprint(1, 16, "13 ALT");
		pMDU->NumberSignBracket(8, 16, TGT_ALT);// TODO should brackets flash with sign?
		sprintf_s(cbuf, 255, "%7.1f", fabs(TGT_ALT));
		pMDU->mvprint(9, 16, cbuf, ALT_flash ? DEUATT_FLASHING : 0);
		pMDU->Underline(9, 16);
		pMDU->Underline(10, 16);
		pMDU->Underline(11, 16);
		pMDU->Underline(12, 16);
		pMDU->Underline(13, 16);
		pMDU->Underline(14, 16);
		pMDU->Underline(15, 16);

		sprintf_s(cbuf, 255, "14 BODY VECT %d", BODV_ID);
		pMDU->mvprint(1, 18, cbuf);
		pMDU->mvprint(1, 20, "15 P");
		sprintf_s(cbuf, 255, "%6.2f", BODV_PITCH / RAD_PER_DEG);
		pMDU->mvprint(7, 20, cbuf, P_flash ? DEUATT_FLASHING : 0);
		pMDU->Underline(7, 20);
		pMDU->Underline(8, 20);
		pMDU->Underline(9, 20);
		pMDU->Underline(10, 20);
		pMDU->Underline(11, 20);
		pMDU->Underline(12, 20);
		pMDU->mvprint(1, 21, "16 Y");
		sprintf_s(cbuf, 255, "%6.2f", BODV_YAW / RAD_PER_DEG);
		pMDU->mvprint(7, 21, cbuf, Y_flash ? DEUATT_FLASHING : 0);

		pMDU->mvprint(1, 22, "17 OM");
		if (OMICRON_flash || THREE_AXIS)
		{
			sprintf_s(cbuf, 255, "%6.2f", OMICRON / RAD_PER_DEG);
			pMDU->mvprint(7, 22, cbuf, OMICRON_flash ? DEUATT_FLASHING : 0);
		}

		pMDU->mvprint(15, 4, "START MNVR 18");
		pMDU->mvprint(21, 5, "TRK  19");
		pMDU->mvprint(21, 6, "ROT  20");
		pMDU->mvprint(20, 7, "CNCL  21");
		pMDU->mvprint(28, 3, "CUR");
		pMDU->mvprint(32, 3, "FUT");

		switch (OPT_CUR)
		{
		case 1:
			pMDU->mvprint(29, 4, "*");
			break;
		case 2:
			pMDU->mvprint(29, 5, "*");
			break;
		case 3:
			pMDU->mvprint(29, 6, "*");
			break;
		}

		switch (OPT_FUT)
		{
		case 1:
			pMDU->mvprint(33, 4, "*");
			break;
		case 2:
			pMDU->mvprint(33, 5, "*");
			break;
		case 3:
			pMDU->mvprint(33, 6, "*");
			break;
		}

		pMDU->mvprint(20, 9, "ATT MON");
		pMDU->mvprint(21, 10, "22 MON AXIS");
		if (MON_AXIS == 1)
		{
			pMDU->mvprint(33, 10, "1 +X");
		}
		else
		{
			pMDU->mvprint(33, 10, "2 -X");
		}
		pMDU->mvprint(21, 11, "ERR TOT 23");
		pMDU->mvprint(21, 12, "ERR DAP 24");
		if (ERR_SEL) pMDU->mvprint(31, 11, "*");// ERR TOT
		else pMDU->mvprint(31, 12, "*");// ERR DAP

		pMDU->mvprint(27, 14, "ROLL   PITCH    YAW");
		sprintf_s(cbuf, 255, "CUR   %6.2f  %6.2f  %6.2f", ROLLC / RAD_PER_DEG, PITCHC / RAD_PER_DEG, YAWC / RAD_PER_DEG);
		pMDU->mvprint(20, 15, cbuf);
		sprintf_s(cbuf, 255, "REQD  %6.2f  %6.2f  %6.2f", ROLLR / RAD_PER_DEG, PITCHR / RAD_PER_DEG, YAWR / RAD_PER_DEG);
		pMDU->mvprint(20, 16, cbuf);
		sprintf_s(cbuf, 255, "ERR   %6.2f  %6.2f  %6.2f", fabs(ATT_ERR.x), fabs(ATT_ERR.y), fabs(ATT_ERR.z));
		pMDU->mvprint(20, 17, cbuf);
		pMDU->NumberSign(25, 17, ATT_ERR.x);
		pMDU->NumberSign(33, 17, ATT_ERR.y);
		pMDU->NumberSign(41, 17, ATT_ERR.z);
		sprintf_s(cbuf, 255, "RATE  %6.3f  %6.3f  %6.3f", fabs(ATT_RATE.x), fabs(ATT_RATE.y), fabs(ATT_RATE.z));
		pMDU->mvprint(20, 18, cbuf);
		pMDU->NumberSign(25, 18, ATT_RATE.x);
		pMDU->NumberSign(33, 18, ATT_RATE.y);
		pMDU->NumberSign(41, 18, ATT_RATE.z);
	}

	bool UniversalPointing::OnParseLine(const char* keyword, const char* value)
	{
		if (!_strnicmp(keyword, "START_TIME", 10)) {
			sscanf_s(value, "%d %d %d %d", &START_TIME[0], &START_TIME[1], &START_TIME[2], &START_TIME[3]);
			return true;
		}
		else if (!_strnicmp(keyword, "MNVR_OPTION", 11)) {
			sscanf_s(value, "%f %f %f", &ROLLI, &PITCHI, &YAWI);
			return true;
		}
		else if (!_strnicmp(keyword, "TGT_ID", 6)) {
			sscanf_s(value, "%d", &TGT_ID);
			return true;
		}
		else if (!_strnicmp(keyword, "TGT_DATA", 8)) {
			sscanf_s(value, "%f %f %f %f %f", &TGT_RA, &TGT_DEC, &TGT_LAT, &TGT_LON, &TGT_ALT);
			return true;
		}
		else if (!_strnicmp(keyword, "BODV_ID", 7)) {
			sscanf_s(value, "%d", &BODV_ID);
			return true;
		}
		else if (!_strnicmp(keyword, "BODV_ATT", 8)) {
			sscanf_s(value, "%f %f %f", &BODV_PITCH, &BODV_YAW, &OMICRON);
			return true;
		}
		else if (!_strnicmp(keyword, "BODV_SCL", 8)) {
			sscanf_s(value, "%f %f", &BODV_SCL[4][0], &BODV_SCL[4][1]);
			return true;
		}
		else if (!_strnicmp(keyword, "FLAGS1", 6)) {
			int inttemp[8];
			sscanf_s(value, "%d %d %d %d %d %d %d %d %d %d %d %d", &inttemp[0], &OPT_CUR, &OPT_FUT, &MON_AXIS, &inttemp[1], &OPT_SEL, &inttemp[2], &inttemp[3], &inttemp[4], &inttemp[5], &inttemp[6], &inttemp[7]);
			THREE_AXIS = (inttemp[0] != 0);
			ERR_SEL = (inttemp[1] != 0);
			OPT_CNCL = (inttemp[2] != 0);
			TRK_DCD = (inttemp[3] != 0);
			ROT_DCD = (inttemp[4] != 0);
			MNV_DCD = (inttemp[5] != 0);
			FUT_DCD = (inttemp[6] != 0);
			PARAM_XFR = (inttemp[7] != 0);
		}
		else if (!_strnicmp(keyword, "FLAGS2", 6)) {
			int inttemp[15];
			sscanf_s(value, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d", &inttemp[0], &inttemp[1], &inttemp[2], &inttemp[3], &inttemp[4], &inttemp[5], &inttemp[6], &inttemp[7], &inttemp[8], &inttemp[9], &inttemp[10], &inttemp[11], &inttemp[12], &inttemp[13], &inttemp[14]);
			ROT_IC = (inttemp[0] != 0);
			MNVR_TIME_IC = (inttemp[1] != 0);
			MNVR_TIME_FLAG = (inttemp[2] != 0);
			RA_flash = (inttemp[3] != 0);
			DEC_flash = (inttemp[4] != 0);
			LAT_flash = (inttemp[5] != 0);
			LON_flash = (inttemp[6] != 0);
			ALT_flash = (inttemp[7] != 0);
			P_flash = (inttemp[8] != 0);
			Y_flash = (inttemp[9] != 0);
			OMICRON_flash = (inttemp[10] != 0);
			PY_executed = (inttemp[11] != 0);
			RA_DEC_executed = (inttemp[12] != 0);
			AUTO_ALERT = (inttemp[13] != 0);
			MNVR_TRACK_OPTION = (inttemp[14] != 0);
		}
		else if (!_strnicmp(keyword, "ACTIVEMANEUVER1", 15)) {
			int inttemp;
			sscanf_s(value, "%lf %lf %lf %d %lf %lf %lf %lf %d %d", &PLOS_C.x, &PLOS_C.y, &PLOS_C.z, &TGT_ID_C, &TGPOS_C.x, &TGPOS_C.y, &TGPOS_C.z, &OMICRON_C, &inttemp, &OPT_PROC);
			THREE_AXIS_C = (inttemp != 0);
		}
		else if (!_strnicmp(keyword, "ACTIVEMANEUVER2", 15)) {
			sscanf_s(value, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &Q_MNVRC_S, &Q_MNVRC_V.x, &Q_MNVRC_V.y, &Q_MNVRC_V.z, &ROTR_RATE_C, &Q_BODC_M50_S, &Q_BODC_M50_V.x, &Q_BODC_M50_V.y, &Q_BODC_M50_V.z, &GMTS_C);
		}
		else if (!_strnicmp(keyword, "FUTUREMANEUVER", 14)) {
			sscanf_s(value, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &MNVR_CMPL_TIME, &GMTS, &Q_MNVR_S, &Q_MNVR_V.x, &Q_MNVR_V.y, &Q_MNVR_V.z, &PLOS.x, &PLOS.y, &PLOS.z, &TGPOS.x, &TGPOS.y, &TGPOS.z);
		}
		else if (!_strnicmp(keyword, "Q_RBOD_M50", 10)) {
			sscanf_s(value, "%lf %lf %lf %lf", &Q_RBOD_M50_S, &Q_RBOD_M50_V.x, &Q_RBOD_M50_V.y, &Q_RBOD_M50_V.z);
		}
		else if (!_strnicmp(keyword, "REQD_BRATE", 10)) {
			sscanf_s(value, "%lf %lf %lf", &REQD_BRATE.x, &REQD_BRATE.y, &REQD_BRATE.z);
		}
		else if (!_strnicmp(keyword, "GMTR", 4)) {
			sscanf_s(value, "%lf", &GMTR);
		}

		return false;
	}

	void UniversalPointing::OnSaveState(FILEHANDLE scn) const
	{
		char cbuf[256];

		sprintf_s(cbuf, 256, "%d %d %d %d", START_TIME[0], START_TIME[1], START_TIME[2], START_TIME[3]);
		oapiWriteScenario_string(scn, "START_TIME", cbuf);
		oapiWriteScenario_vec(scn, "MNVR_OPTION", _V(ROLLI, PITCHI, YAWI));
		oapiWriteScenario_int(scn, "TGT_ID", TGT_ID);
		sprintf_s(cbuf, 256, "%f %f %f %f %f", TGT_RA, TGT_DEC, TGT_LAT, TGT_LON, TGT_ALT);
		oapiWriteScenario_string(scn, "TGT_DATA", cbuf);
		oapiWriteScenario_int(scn, "BODV_ID", BODV_ID);
		oapiWriteScenario_vec(scn, "BODV_ATT", _V(BODV_PITCH, BODV_YAW, OMICRON));
		sprintf_s(cbuf, 256, "%f %f", BODV_SCL[4][0], BODV_SCL[4][1]);
		oapiWriteScenario_string(scn, "BODV_SCL", cbuf);
		sprintf_s(cbuf, 256, "%d %d %d %d %d %d %d %d %d %d %d %d", THREE_AXIS, OPT_CUR, OPT_FUT, MON_AXIS, ERR_SEL, OPT_SEL, OPT_CNCL, TRK_DCD, ROT_DCD, MNV_DCD, FUT_DCD, PARAM_XFR);
		oapiWriteScenario_string(scn, "FLAGS1", cbuf);
		sprintf_s(cbuf, 256, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d", ROT_IC, MNVR_TIME_IC, MNVR_TIME_FLAG, RA_flash, DEC_flash, LAT_flash, LON_flash, ALT_flash, P_flash, Y_flash, OMICRON_flash, PY_executed, RA_DEC_executed, AUTO_ALERT, MNVR_TRACK_OPTION);
		oapiWriteScenario_string(scn, "FLAGS2", cbuf);
		sprintf_s(cbuf, 256, "%lf %lf %lf %d %lf %lf %lf %lf %d %d", PLOS_C.x, PLOS_C.y, PLOS_C.z, TGT_ID_C, TGPOS_C.x, TGPOS_C.y, TGPOS_C.z, OMICRON_C, THREE_AXIS_C, OPT_PROC);
		oapiWriteScenario_string(scn, "ACTIVEMANEUVER1", cbuf);
		sprintf_s(cbuf, 256, "%.9lf %.9lf %.9lf %.9lf %lf %.9lf %.9lf %.9lf %.9lf %lf", Q_MNVRC_S, Q_MNVRC_V.x, Q_MNVRC_V.y, Q_MNVRC_V.z, ROTR_RATE_C, Q_BODC_M50_S, Q_BODC_M50_V.x, Q_BODC_M50_V.y, Q_BODC_M50_V.z, GMTS_C);
		oapiWriteScenario_string(scn, "ACTIVEMANEUVER2", cbuf);
		sprintf_s(cbuf, 256, "%lf %lf %.9lf %.9lf %.9lf %.9lf %lf %lf %lf %lf %lf %lf", MNVR_CMPL_TIME, GMTS, Q_MNVR_S, Q_MNVR_V.x, Q_MNVR_V.y, Q_MNVR_V.z, PLOS.x, PLOS.y, PLOS.z, TGPOS.x, TGPOS.y, TGPOS.z);
		oapiWriteScenario_string(scn, "FUTUREMANEUVER", cbuf);
		sprintf_s(cbuf, 256, "%.9lf %.9lf %.9lf %.9lf", Q_RBOD_M50_S, Q_RBOD_M50_V.x, Q_RBOD_M50_V.y, Q_RBOD_M50_V.z);
		oapiWriteScenario_string(scn, "Q_RBOD_M50", cbuf);
		oapiWriteScenario_vec(scn, "REQD_BRATE", REQD_BRATE);
		oapiWriteScenario_float(scn, "GMTR", GMTR);
	}

	void UniversalPointing::KLoad()
	{
		for (unsigned int i = 0; i < 4; i++) START_TIME[i] = 0;
		ROLLI = PITCHI = YAWI = 0.0f;
		TGT_ID = 2;
		TGT_RA = TGT_DEC = 0.0f;
		TGT_LAT = TGT_LON = TGT_ALT = 0.0f;
		BODV_ID = 1;
		BODV_PITCH = BODV_YAW = OMICRON = 0.0f;
		THREE_AXIS = true;
		OPT_CUR = 0;
		OPT_FUT = 0;
		MON_AXIS = 1;
		ERR_SEL = true;

		OPT_PROC = 0;
		ROT_IC = false;
		PARAM_XFR = false;
		BODV_SCL[4][0] = 0.0; BODV_SCL[4][1] = 0.0;
		I_STAR_SEL_5 = _V(1, 0, 0);

		MNVR_TRACK_OPTION = false;
	}

	void UniversalPointing::UNIV_SEQ()
	{
		// The PARAM PROC task is performed each time a reboost request or an attitude option related item
		// on the universal pointing display is executed.Therefore, each time a data change discrete is set, an attitude
		// option is selected, a reboost request is executed or the maneuver cancel option is selected, the parameter
		// processing task is executed.
		if (MNV_DCD || TRK_DCD || ROT_DCD || FUT_DCD || OPT_SEL != 0 || OPT_CNCL) //TBD: RBST_RQST
		{
			PARAM_PROC();
		}

		// The starting GMT is then relative to the current GMT to determine when to initiate the selected attitude option
		if (GMT > GMTS)
		{
			if (OPT_FUT != 0)
			{
				if (OPT_CUR != 0)
				{
					PARAM_XFR = true;
				}
				else
				{
					if (OPT_FUT == 3)
					{
						PARAM_XFR = true;
					}
				}

				OPT_CUR = OPT_FUT;
				OPT_FUT = 0;
				MNVR_TRACK_OPTION = true;
				OPT_PROC = OPT_CUR;
				ROT_IC = true;
				MNVR_TIME_IC = true;
				MNVR_TIME_FLAG = true;

				// TBD: Orbit DAP should check MNVR_TRACK_OPTION instead
				pOrbitDAP->ManeuverToUPAttitude();
			}
		}

		// LVLH_MODE, is tested to determine if the cyclic universal pointing attitude options should be inhibited and the cyclic LVLH attitude mode task performed
		if (pOrbitDAP->GetDAPMode() == OrbitDAP::DAP_CONTROL_MODE::LVLH)
		{
			CYC_LVLH();
			ROT_IC = true;
		}
		else
		{
			if (pOrbitDAP->Get_Preburn_Mnvr_In_Progress() == false)
			{
				if (OPT_PROC != 0)
				{
					CYC_OPT();
				}
			}
		}
		if (OPT_FUT != 0)
		{
			if (pOrbitDAP->GetDAPMode() != OrbitDAP::DAP_CONTROL_MODE::AUTO)
			{
				if (GMTS - GMT <= 30.0)
				{
					AUTO_ALERT = true;
				}
			}
		}
		if (pOrbitDAP->GetDAPMode() == OrbitDAP::DAP_CONTROL_MODE::AUTO)
		{
			AUTO_ALERT = false;
		}
		if (OPT_CUR != 0)
		{
			if (OPT_FUT == 0)
			{
				AUTO_ALERT = false;
			}
		}
		//if (KU_SEL == true)
		//{
		//	RNDZTGT();
		//}
		if (OPT_CUR != 0)
		{
			if (pOrbitDAP->GetDAPMode() != OrbitDAP::DAP_CONTROL_MODE::AUTO)
			{
				MNVR_TIME_IC = true;
				MNVR_TIME_FLAG = true;
			}
		}
		// TBD: Reboost control
		DISP_SUP();
	}

	void UniversalPointing::PARAM_PROC()
	{
		double METS;

		// If an attitude option is selected, OPT_SEL != 0, the future attitude option indicator is set and the selected attitude option indicator is reset
		if (OPT_SEL != 0)
		{
			OPT_FUT = OPT_SEL;
			OPT_SEL = 0;

			// If a future maneuver option is selected, OPT_FUT = 1, the maneuver option quaternion subtask is called to calculate the maneuver option quaternion
			if (OPT_FUT == 1)
			{
				MNVR_QUAT();
			}
			// If a future tracking or rotation option is selected, OPT_FUT = 2 or 3, the body vector subtask is called to calculate the body vector
			else
			{
				BODY_VEC();
				// If a future tracking option is selected, OPT_FUT = 2, and TGT_ID = 3, the earth–fixed target position vector subtask is called
				if (OPT_FUT == 2)
				{
					if (TGT_ID == 3)
					{
						ETGT_POS();
					}
				}
			}
			// If no current option is set (OPT_CUR = 0), then perform the following:
			if (OPT_CUR == 0)
			{
				// If no future rotation option is set (OPT_FUT != 3), then set the option process indicator and the parameter transfer indicator:
				if (OPT_FUT != 3)
				{
					OPT_PROC = OPT_FUT;
					PARAM_XFR = true;
				}
				// Otherwise, if a future rotation option is set (OPT_FUT = 3), then reset the option process indicator:
				else
				{
					OPT_PROC = false;
				}
			}
		}
		// Attitude Option Start Time. The attitude option start time is computed in seconds of Greenwich mean time (GMT).First, the start time is computed in seconds, mission elapsed time(MET):
		METS = 86400.0 * START_TIME[0] + 3600.0 * START_TIME[1] + 60.0 * START_TIME[2] + START_TIME[3];
		// Second, METS is converted to GMT by adding the MET/GMT reference time to METS:
		GMTS = METS + ReadCOMPOOL_SD(SCP_T_MET_REF);
		// Maneuver Option Data Change Discrete. If the maneuver option data change discrete is set, MNV_DCD = 1, and a future maneuver option is
		// selected, OPT_FUT = 1, then set the future option data change discrete:
		if (MNV_DCD)
		{
			if (OPT_FUT == 1)
			{
				FUT_DCD = true;
			}
		}
		// If the tracking option data change discrete is set, TRK_DCD = 1, several parameters are tested and set.
		if (TRK_DCD)
		{
			VECTOR3 TGT_VEC;

			// First, if a future tracking option is selected, then set the future option data change discrete:
			if (OPT_FUT == 2)
			{
				FUT_DCD = true;
			}
			// Second, if TGT_ID = 5 or 11 <= TGT_ID <= 110, and if no target RA or DEC has been executed, the celestial target RA/DEC subtask is called:
			if (TGT_ID == 5 || (TGT_ID >= 11 && TGT_ID <= 110))
			{
				if (RA_DEC_executed == false)
				{
					RA_DEC(TGT_VEC);
				}
			}
			// Third, if TGT_ID = 4, the celestial target RA/DEC subtask is called:
			if (TGT_ID == 4)
			{
				TGT_VEC = ReadCOMPOOL_VD(SCP_UR_SUN);
				RA_DEC(TGT_VEC);
			}
			// Fourth, if TGT_ID = 5, the celestial target vector subtask is called:
			if (TGT_ID == 5)
			{
				STAR();
			}
		}
		// If the tracking TRK_DCD = 1, or rotation option data change discrete is set, ROT_DCD = 1, several parameters are computed.
		if (TRK_DCD || ROT_DCD)
		{
			// First, if no body vector pitch or yaw has been executed, the body vector pitch/yaw subtask is called:
			if (PY_executed == false)
			{
				PY();
			}
			// Then, if BODV_ID = 5, the body vector pitch and yaw components are stored in the body vector table:
			if (BODV_ID == 5)
			{
				BODV_SCL[4][0] = BODV_PITCH;
				BODV_SCL[4][1] = BODV_YAW;
			}
		}
		// If the rotation option data change discrete is set, ROT_DCD = 1, and a future rotation option is selected, OPT_FUT = 3, then
		// set the future option data change discrete:
		if (ROT_DCD)
		{
			if (OPT_FUT == 3)
			{
				FUT_DCD = true;
			}
		}
		// If the future option data change discrete is set, FUT_DCD = 1, then perform the following:
		if (FUT_DCD)
		{
			// Reset the future attitude option indicator:
			OPT_FUT = 0;
			// If no current option is set (OPT_CUR = 0), the following equations are executed:
			if (OPT_CUR == 0)
			{
				OPT_PROC = 0;
				AUTO_ALERT = false;
				Q_RBOD_M50_S = Q_BOD_M50_S;
				Q_RBOD_M50_V = Q_BOD_M50_V;
			}
		}
		// All data change discretes are reset before exiting the PARAM PROC task.
		MNV_DCD = TRK_DCD = ROT_DCD = FUT_DCD = false;
		PY_executed = RA_DEC_executed = false; //TBD: Is this right?
		// If OPT_CNCL is equal to one, the cancel option has been selected. The following equations are then executed:
		if (OPT_CNCL)
		{
			Q_RBOD_M50_S = Q_BOD_M50_S;
			Q_RBOD_M50_V = Q_BOD_M50_V;
			OPT_CNCL = false;
			OPT_CUR = false;
			OPT_FUT = false;
			MNVR_TRACK_OPTION = false;
			OPT_PROC = false;
			AUTO_ALERT = false;
			MNVR_CMPL_TIME = 0.0;

			// TBD: Orbit DAP should check MNVR_TRACK_OPTION instead
			pOrbitDAP->CancelManeuver();
		}
	}

	void UniversalPointing::MNVR_QUAT()
	{
		// This subtask calculates the inertial body quaternion, Q_MNVR, required by the MNVR attitude option.

		VECTOR3 Q_RBOD_ADI_V;
		double CR, SR, CP, SP, CY, SY, Q_RBOD_ADI_S;

		CR = cos(ROLLI / 2.0);
		SR = sin(ROLLI / 2.0);
		CP = cos(PITCHI / 2.0);
		SP = sin(PITCHI / 2.0);
		CY = cos(YAWI / 2.0);
		SY = sin(YAWI / 2.0);

		Q_RBOD_ADI_S = CR * CP * CY - SR * SP * SY;
		Q_RBOD_ADI_V = _V(-SR * CP * CY - CR * SP * SY, -CR * SP * CY - SR * CP * SY, -CR * CP * SY + SR * SP * CY);
		QUAT_MULT(Q_RBOD_ADI_S, Q_RBOD_ADI_V, Q_M50_INRTL_ORB_S, -Q_M50_INRTL_ORB_V, Q_MNVR_S, Q_MNVR_V);
	}

	void UniversalPointing::STAR()
	{
		// This subtask computes the inertially referenced unit vector, TGT_VEC, required for celestial target pointing

		VECTOR3 TGT_VEC = _V(cos(TGT_RA) * cos(TGT_DEC), sin(TGT_RA) * cos(TGT_DEC), sin(TGT_DEC));
		I_STAR_SEL_5 = TGT_VEC;
	}

	void UniversalPointing::RA_DEC(VECTOR3& TGT_VEC)
	{
		// This subtask calculates the right ascension (TGT_RA) and declination (TGT_DEC) of celestial targets for the universal pointing display

		if (TGT_ID == 5)
		{
			TGT_VEC = I_STAR_SEL_5;
		}
		else if (TGT_ID >= 11 && TGT_ID <= 110)
		{
			TGT_VEC = I_STAR_SEL[TGT_ID - 11];
		}
		double temp = atan2(TGT_VEC.y, TGT_VEC.x);
		if (temp < 0.0) temp += PI2;
		TGT_RA = (float)temp;
		TGT_DEC = (float)asin(TGT_VEC.z);
	}

	void UniversalPointing::BODY_VEC()
	{
		// This subtask computes the body referenced unit vector, PLOS, required by the tracking and rotation attitude options

		double PITCH, YAW;

		PITCH = BODV_PITCH;
		YAW = BODV_YAW;

		PLOS = _V(cos(PITCH) * cos(YAW), sin(YAW), -sin(PITCH) * cos(YAW));
	}

	void UniversalPointing::PY()
	{
		// This subtask obtains the pitch (BODV_PITCH) and yaw (BODV_YAW) coordinates of the selected body vector required by the universal pointing display

		BODV_PITCH = BODV_SCL[BODV_ID - 1][0];
		BODV_YAW = BODV_SCL[BODV_ID - 1][1];
	}

	void UniversalPointing::ETGT_POS()
	{
		//This subtask computes the earth-fixed or Greenwich true-of-date position vector, TGPOS, required for earth surface pointing

		double ALT, TEMP_1, TEMP_2, TEMP_3;

		ALT = TGT_ALT / NAUTMI_PER_FT;
		TEMP_1 = pow(1.0 - ELLIPT, 2);
		TEMP_2 = EARTH_RADIUS_EQUATOR / sqrt(pow(cos(TGT_LAT), 2) + TEMP_1 * pow(sin(TGT_LAT), 2));
		TEMP_3 = (TEMP_2 + ALT) * cos(TGT_LAT);
		TGPOS = _V(TEMP_3 * cos(TGT_LON), TEMP_3 * sin(TGT_LON), (TEMP_1 * TEMP_2 + ALT) * sin(TGT_LAT));
	}

	void UniversalPointing::CYC_LVLH()
	{
		// This task provides the local vertical, local horizontal body attitude rate vectors required for proximity operations.
		// The outputs of this task support the DAP manual LVLH attitude mode.

		int TEMP_ID;

		TEMP_ID = TGT_ID_C;
		TGT_ID_C = 2;

		LOS_VEC();
		TRK_RATE();

		TGT_ID_C = TEMP_ID;
	}

	void UniversalPointing::CYC_OPT()
	{
		// The parameter transfer flag is a discrete that controls the initialization of the chosen attitude option.If PARAM_XFR = 1, then the parameters from UNIV PTG and
		// the PARAM PROC task are transferred to a set of locations allocated for current processing.
		if (PARAM_XFR)
		{
			PLOS_C = PLOS;
			TGT_ID_C = TGT_ID;
			TGPOS_C = TGPOS;
			OMICRON_C = OMICRON;
			THREE_AXIS_C = THREE_AXIS;
			Q_MNVRC_S = Q_MNVR_S;
			Q_MNVRC_V = Q_MNVR_V;
			PARAM_XFR = false;
		}

		// Processing of an attitude option is required for either a current option or a future tracking or maneuver attitude option when there is no current option.
		// Cyclic processing for the future option is required to support the computation of total attitude errors.
		// A single parameter, OPT_PROC, indicates which option is currently being processed
		if (OPT_PROC == 1)
		{
			MNVR();
		}
		else if (OPT_PROC == 2)
		{
			TRACK();
		}
		else
		{
			if (pOrbitDAP->GetDAPMode() == OrbitDAP::DAP_CONTROL_MODE::AUTO)
			{
				ROTR();
			}
			else
			{
				ROT_IC = true;
			}
		}
	}

	void UniversalPointing::MNVR()
	{
		// This task provides the required attitude to attain a specified set of pitch-yaw-roll Euler angles input
		// through the UNIV PTG display function.The angles are referenced to the inertial ADI coordinate system
		Q_RBOD_M50_S = Q_MNVRC_S;
		Q_RBOD_M50_V = Q_MNVRC_V;

		//No attitude rate computation is required for this attitude task, since the DAP maneuver rate will be used.
		REQD_BRATE = _V(0, 0, 0);
	}

	void UniversalPointing::TRACK()
	{
		// This task provides the body attitude and body attitude rate vector required to align the orbiter body vector
		// with a target vector.Two attitude alignment modes are available: 2AXIS and 3AXIS. The 2AXIS mode
		// provides a minimum attitude change in which the maneuver eigenangle is minimized.The 3AXIS mode
		// provides for an attitude constraint by specifying a fixed angle of rotation about the required pointing line-of-sight

		// The target vector is either computed or obtained as a function of the target or
		// TGT vector identifier(TGT_ID_C) from I-load locations common to both the Star Tracker
		// SOP and this function. The target line-of-sight vector subtask is used to produce TLOS.
		LOS_VEC();

		if (THREE_AXIS_C == false)
		{
			// If THREE_AXIS_C = 0, then the 2AXIS tracking mode is selected. This tracking mode computes the minimum eigenangle required to align
			// PLOS_C with TLOS. The 2AXIS tracking subtask is called to obtain the required body quaternion.
			F2AXIS(PLOS_C, TLOS);
		}
		else
		{
			// If the angle OMICRON_C is specified (THREE_AXIS_C = 1), then the 3AXIS tracking mode is selected
			VECTOR3 RRA_BOD;
			double ROLL;

			RR_BOD = _V(0, 1, 0);
			RR_M50 = -unit(crossp(RORB, VORB));
			RRA_BOD = _V(0, 0, -1);
			if (abs(dotp(PLOS_C, RR_BOD)) > PLOS_TOL)
			{
				RR_BOD = RRA_BOD;
			}
			RRA_M50 = crossp(_V(0, 0, 1), RR_M50);
			ROLL = OMICRON_C + PI05;

			F3AXIS(PLOS_C, TLOS, ROLL, Q_RBOD_M50_S, Q_RBOD_M50_V);
		}

		// A body attitude rate vector is required by the DAP AUTO_MNVR_TRACK function for target tracking.
		// The computation of this rate vector is based on the type of target selected.
		// This vector is computed by the tracking rate vector subtask and expressed in body axes.
		TRK_RATE();

		// The DAP requires a time tag associated with the required body quaternion and attitude rate vector in order to remove phase plane bias errors
		// during target tracking.The time tag will be the time of the attitude input to the UNIV POINT PROC from the attitude processor.
		GMTR = GMTC;
	}

	void UniversalPointing::LOS_VEC()
	{
		// This subtask computes the target line-of-sight vector, TLOS, required by the tracking task

		if (TGT_ID_C == 1) // Rendezvous
		{
			TLOS = unit(RREL);
		}
		else if (TGT_ID_C == 2) // Center of earth
		{
			TLOS = unit(-RORB);
		}
		else if (TGT_ID_C == 3) // Earth site
		{
			MATRIX3 EF_TO_M50;
			VECTOR3 EARTH_POLE;

			EF_TO_M50 = pGNCUtilities->EARTH_FIXED_TO_M50_COORD(GMTC);
			EARTH_POLE = _V(EF_TO_M50.m13, EF_TO_M50.m23, EF_TO_M50.m33);
			TPOS = mul(EF_TO_M50, TGPOS_C);
			TLOS = unit(TPOS - RORB);
		}
		else if (TGT_ID_C == 4) // Center of the sun
		{
			TLOS = ReadCOMPOOL_VD(SCP_UR_SUN);
		}
		else if (TGT_ID_C == 5) // Celestial target
		{
			TLOS = I_STAR_SEL_5;
		}
		else if (TGT_ID_C >= 11 && TGT_ID_C <= 110)
		{
			TLOS = I_STAR_SEL[TGT_ID_C - 11];
		}
	}

	void UniversalPointing::F2AXIS(VECTOR3 P, VECTOR3 T)
	{
		// This subtask computes the required body quaternion that will align a given body referenced vector, P, with
		// a given inertial referenced vector, T, through a minimum attitude maneuver.

		VECTOR3 TB, Q_RBOD_BOD_V;
		double DOT, Q_RBOD_BOD_S;

		// Since the vector 7 is expressed in the M50 coordinate system, it must be transformed into the body coordinate system.
		// The transformation is accomplished through the QUAT_XFORM module, where TB is the target vector expressed in the body coordinate system.
		TB = QUAT_XFORM(Q_BOD_M50_S, Q_BOD_M50_V, T);

		// If P and TB are nearly 180 degrees apart, a rotation axis is chosen as near as possible to the body X-axis:
		DOT = dotp(P, TB);
		if (DOT < -0.9999996)
		{
			double MAG;

			Q_RBOD_BOD_S = 0.0;
			MAG = 1.0 - P.x * P.x;
			if (MAG < 0.0000008)
			{
				Q_RBOD_BOD_V = _V(0, -1, 0);
			}
			else
			{
				MAG = sqrt(MAG);
				Q_RBOD_BOD_V = _V(-MAG, P.x * P.y / MAG, P.x * P.z / MAG);
			}
		}
		else
		{
			VECTOR3 LAMBDA;

			LAMBDA = (P + TB) / sqrt(2.0 + 2.0 * DOT);
			Q_RBOD_BOD_S = dotp(LAMBDA, P);
			Q_RBOD_BOD_V = crossp(LAMBDA, P);
		}

		// This body-to-required-body quaternion is used to produce the required body quaternion:
		QUAT_MULT(Q_RBOD_BOD_S, Q_RBOD_BOD_V, Q_BOD_M50_S, Q_BOD_M50_V, Q_RBOD_M50_S, Q_RBOD_M50_V);
	}

	void UniversalPointing::F3AXIS(VECTOR3 VEC_BOD, VECTOR3 VEC_M50, double ROLL, double& Q_ATT_M50_S, VECTOR3& Q_ATT_M50_V)
	{
		// This subtask computes a quaternion that will align a given body referenced vector, VEC_BOD, with a given inertial vector, VEC_M50,
		// such that a specified attitude constraint is satisfied. The attitude constraint is specified by a roll angle, ROLL, and
		// two roll reference vectors: a body roll reference vector, RR_BOD, and an inertial roll reference vector, RR_M50. An alternate inertial roll reference vector,
		// RRA_M50, must be specified to avoid inherent software singularities when VEC_M50 is within a given tolerance of RR_M50.

		MATRIX3 MTP;
		VECTOR3 YN, YT;
		double DOT;

		// Initially, a test is performed to determine if the alternate inertial roll reference
		// vector is to be used in computing the quaternion.First, the dot product of the inertial vector and the inertial
		// roll reference vector is taken:
		DOT = dotp(VEC_M50, RR_M50);
		if (abs(DOT) > TLOS_TOL)
		{
			RR_M50 = RRA_M50 * sign(DOT);
		}

		// The direction cosine matrix transformation from the M50 system to the required body system is computed as follows:
		YN = unit(crossp(VEC_BOD, RR_BOD));
		YT = unit(crossp(VEC_M50, RR_M50)) * sin(ROLL) - crossp(VEC_M50, unit(crossp(VEC_M50, RR_M50))) * cos(ROLL);
		MTP = mul(Transpose(MATRIX(VEC_BOD, crossp(VEC_BOD, YN), -YN)), MATRIX(VEC_M50, crossp(VEC_M50, YT), -YT));

		// The required quaternion is computed from the MTP matrix by calling the MAT_TO_QUAT utility function:
		MAT_TO_QUAT(MTP, Q_ATT_M50_S, Q_ATT_M50_V);
	}

	void UniversalPointing::TRK_RATE()
	{
		// This subtask calculates the tracking rate vector, REQD_BRATE, required by the DAP AUTO_MNVR_TRACK function for target tracking

		// If TGT_ID_C >= 4, set REQD_BRATE = 0. For all other targets, first, the inertial rate vector, WBI, is calculated as a function of the target selected
		if (TGT_ID_C >= 4)
		{
			REQD_BRATE = _V(0, 0, 0);
		}
		else
		{
			VECTOR3 WBI;

			if (TGT_ID_C == 1) // Rendezvous
			{
				WBI = crossp(TLOS, VREL) / max(length(RREL), RNG_TOL_UPTG);
			}
			else if (TGT_ID_C == 2) // Center of Earth
			{
				WBI = crossp(VORB, TLOS) / length(RORB);
			}
			else if (TGT_ID_C == 3) // Earth site
			{
				VECTOR3 VPOS;

				VPOS = crossp(ReadCOMPOOL_VD(SCP_EARTH_POLE), TPOS) * EARTH_RATE;
				WBI = crossp(TLOS, VPOS - VORB) / max(length(TPOS - RORB), RNG_TOL_UPTG);
			}

			// If the three-axis tracking mode is selected(THREE_AXIS_C = 1) and the LVLH mode is not selected(LVLH_ MODE = 0),
			// a correction term must be added to the inertial rate vector to satisfy the roll constraint
			if (THREE_AXIS_C && pOrbitDAP->GetDAPMode() != OrbitDAP::DAP_CONTROL_MODE::LVLH)
			{
				double DOT;

				DOT = dotp(TLOS, RR_M50);
				WBI = WBI + TLOS * dotp(WBI, RR_M50) * DOT / sqrt(1.0 - DOT * DOT);
			}
			// The required body attitude rate vector is then computed
			REQD_BRATE = QUAT_XFORM(Q_BOD_M50_S, Q_BOD_M50_V, WBI);
			// Finally, the required body attitude rate vector is scaled to units of degrees per second
			REQD_BRATE = REQD_BRATE * DEG;
			// If the magnitude of the required body attitude rate vector, ABVAL(REQD_BRATE), is greater than RTE_LIM,
			// then each component of the rate vector is scaled down such that the resulting magnitude is equal to RTE_LIM.
			if (length(REQD_BRATE) > RTE_LIM)
			{
				REQD_BRATE = REQD_BRATE * RTE_LIM / length(REQD_BRATE);
			}
		}
	}

	void UniversalPointing::ROTR()
	{
		// This task provides the body attitude and body attitude rate required to maintain a predictable and continual attitude change about a selected axis of rotation

		double EA, Q_RBOD_BOD_S;
		VECTOR3 Q_RBOD_BOD_V;

		// To initialize, the ROTR task stores four parameters if the rotation option initial condition indicator, ROT_IC, is set
		// or if the current rotation rate, ROTR_RATE_C, is not equal to the selected DAP maneuver rate, DAP_RATE.
		if (ROT_IC || ROTR_RATE_C != pOrbitDAP->GetDAPRate())
		{
			//The first parameter stored is the selected DAP rate
			ROTR_RATE_C = pOrbitDAP->GetDAPRate();
			// The second parameter stored is the starting attitude quaternion
			Q_BODC_M50_S = Q_BOD_M50_S;
			Q_BODC_M50_V = Q_BOD_M50_V;
			// The third stored parameter is the body attitude rate vector, where PLOS_C is the axis of rotation
			REQD_BRATE = PLOS_C * ROTR_RATE_C;
			// And the fourth parameter stored is the activation time
			GMTS_C = GMTC;
			// Also for initialization, the calculation of the maneuver completion time should be initialized to register the rotation option start time
			MNVR_TIME_IC = true;
			MNVR_TIME_FLAG = true;
			//To complete initialization, ROT_IC is reset
			ROT_IC = false;
		}

		// The time-varying parameters are updated on each pass

		// On the first pass and each pass thereafter, the required eigenangle(EA) is computed from the current
		// rotation rate (ROTR_RATE_C), the time of the current attitude(GMTC), and the current attitude option
		// start time (GMTS_C):
		EA = (GMTC - GMTS_C) * ROTR_RATE_C;
		EA = fmod(EA, 360.0) * RAD;
		//Next, the body-to-required-body quaternion is computed
		Q_RBOD_BOD_S = cos(EA / 2.0);
		Q_RBOD_BOD_V = -PLOS_C * sin(EA / 2.0);
		// Then the required body quaternion is computed
		QUAT_MULT(Q_RBOD_BOD_S, Q_RBOD_BOD_V, Q_BODC_M50_S, Q_BODC_M50_V, Q_RBOD_M50_S, Q_RBOD_M50_V);
		// The time tag of the output quaternion is then computed
		GMTR = GMTC;
	}

	void UniversalPointing::DISP_SUP()
	{
		//This task provides all of the dynamic display variables that appear on the attitude monitor portion of the UNIV PTG display

		VECTOR3 ERR, MON_AXIS_VEC, Q_MON_BOD_V, Q_MON_M50_V, BOD_RATE, Q_MON_INRTL_V;
		double Q_MON_BOD_S, Q_MON_M50_S, Q_MON_INRTL_S;

		if (pOrbitDAP->GetDAPMode() == OrbitDAP::DAP_CONTROL_MODE::LVLH)
		{
			//If the LVLH mode indicator is set (LVLH_MODE = 1), the required body quaternion is set equal to the current body quaternion
			//This causes zero total attitude errors to appear on the display and on the ADI error needles
			Q_RBOD_M50_S = Q_BOD_M50_S;
			Q_RBOD_M50_V = Q_BOD_M50_V;
		}

		//Two types of mutually exclusive attitude errors may be selected: total attitude errors and DAP attitude errors
		//Total attitude errors are defined as the negative of the product of the eigenangle and eigenaxis extracted from the body-to-required-body quaternion.
		//DAP attitude errors are defined as the phase plane attitude errors. The total attitude error subtask is used to compute the "fly to" total errors
		ATT_ERROR();

		//The selection of total or DAP attitude errors is indicated by the total/DAP attitude error selection discrete.
		if (ERR_SEL == false)
		{
			ERR = pOrbitDAP->GetAttitudeErrors();
		}
		else
		{
			ERR = TOT_ERR;
		}

		//The selected monitor axis, MON_AXIS, controls the sense in which certain display parameters are computed
		if (MON_AXIS == 1)
		{
			MON_AXIS_VEC = _V(1.0, 1.0, 1.0);
			Q_MON_BOD_S = 1.0;
			Q_MON_BOD_V = _V(0, 0, 0);
		}
		else
		{
			MON_AXIS_VEC = _V(-1.0, -1.0, -1.0);
			Q_MON_BOD_S = 0.0;
			Q_MON_BOD_V = _V(0, 0, 1.0);
		}

		//Calculate current attitude angles
		QUAT_MULT(Q_MON_BOD_S, Q_MON_BOD_V, Q_BOD_M50_S, Q_BOD_M50_V, Q_MON_M50_S, Q_MON_M50_V);
		QUAT_MULT(Q_MON_M50_S, Q_MON_M50_V, Q_M50_INRTL_ORB_S, Q_M50_INRTL_ORB_V, Q_MON_INRTL_S, Q_MON_INRTL_V);
		ATT_ANG(Q_MON_INRTL_S, Q_MON_INRTL_V, ROLLC, PITCHC, YAWC);

		//Calculate required attitude angles
		QUAT_MULT(Q_MON_BOD_S, Q_MON_BOD_V, Q_RBOD_M50_S, Q_RBOD_M50_V, Q_MON_M50_S, Q_MON_M50_V);
		QUAT_MULT(Q_MON_M50_S, Q_MON_M50_V, Q_M50_INRTL_ORB_S, Q_M50_INRTL_ORB_V, Q_MON_INRTL_S, Q_MON_INRTL_V);
		ATT_ANG(Q_MON_INRTL_S, Q_MON_INRTL_V, ROLLR, PITCHR, YAWR);

		//Calculate displayed attitude error
		ATT_ERR = _V(ERR.x * MON_AXIS_VEC.x, ERR.y * MON_AXIS_VEC.y, ERR.z * MON_AXIS_VEC.z);

		//Calculate displayed attitude rate
		BOD_RATE = pOrbitDAP->Get_RATE_EST();
		ATT_RATE = _V(BOD_RATE.x * MON_AXIS_VEC.x, BOD_RATE.y * MON_AXIS_VEC.y, BOD_RATE.z * MON_AXIS_VEC.z);

		//Calculate maneuver completion time
		if (OPT_CUR != 0)
		{
			if (pOrbitDAP->GetDAPMode() != OrbitDAP::DAP_CONTROL_MODE::LVLH)
			{
				CMPLT_TIME();
			}
		}
	}

	void UniversalPointing::ATT_ANG(double QS, VECTOR3 QV, float& ROLL, float& PITCH, float& YAW) const
	{
		// This subtask computes a unique Euler angle representation of a given quaternion.The Euler angles are
		// computed for the pitch-yaw-roll (YZX) attitude sequence. The yaw angle is restricted to a range of values
		// from either 0 to 90 degrees or 270 to 359.995 degrees

		double SYAW, CYAW, SROLL, CROLL, SPITCH, CPITCH;

		// The sine (SYAW) and cosine (CYAW) functions of the yaw attitude angle (YAW) are first computed
		SYAW = 2.0 * (QV.x * QV.y - QV.z * QS);
		CYAW = sqrt(1.0 - SYAW * SYAW);

		if (CYAW > 0.005)
		{
			// If the cosine of the yaw angle is greater than a given tolerance (CYAW > TOL), then the roll and pitch
			// arguments of the four-quadrant arc tangent function are computed as follows:

			SROLL = -2.0 * (QV.y * QV.z + QV.x * QS);
			CROLL = 1.0 - 2.0 * (QV.x * QV.x + QV.z * QV.z);
			SPITCH = -2.0 * (QV.x * QV.z + QV.y * QS);
			CPITCH = 1.0 - 2.0 * (QV.y * QV.y + QV.z * QV.z);
		}
		else
		{
			// If the cosine of the yaw angle is less than or equal to a given tolerance (CYAW <= TOL), then the roll and
			// pitch arc tangent arguments are computed as follows:
			SROLL = 0.0;
			CROLL = 1.0;
			SPITCH = 2.0 * (QV.x * QV.z - QV.y * QS);
			CPITCH = 1.0 - 2.0 * (QV.x * QV.x + QV.y * QV.y);
		}

		// The attitude angles are then computed in radians:
		PITCH = (float)(atan2(-SPITCH, -CPITCH) + PI);
		YAW = (float)(atan2(-SYAW, -CYAW) + PI);
		ROLL = (float)(atan2(-SROLL, -CROLL) + PI);

		// If any of the above angles are greater than 359.995 degrees (6.283098 radians) after scaling, the angles are
		// set to zero. This will ensure that all displayed attitude angles will assume values from 0 to 359.99 degrees
		if (PITCH >= 6.283098f) PITCH = 0.0f;
		if (YAW >= 6.283098f) YAW = 0.0f;
		if (ROLL >= 6.283098f) ROLL = 0.0f;
	}

	void UniversalPointing::ATT_ERROR()
	{
		// This subtask computes the total attitude error, TOT_ERR, required by the universal pointing display function

		double Q_RBOD_BOD_S, VMAG;
		VECTOR3 Q_RBOD_BOD_V;

		// Compute error quaternion Q_RBOD_BOD
		QUAT_MULT(Q_RBOD_M50_S, Q_RBOD_M50_V, Q_BOD_M50_S, -Q_BOD_M50_V, Q_RBOD_BOD_S, Q_RBOD_BOD_V);

		// If Q_RBOD_BOD_S is negative, then the sign of each element of Q_RBOD_BOD is reversed
		if (Q_RBOD_BOD_S < 0.0)
		{
			Q_RBOD_BOD_S = -Q_RBOD_BOD_S;
			Q_RBOD_BOD_V = -Q_RBOD_BOD_V;
		}

		if (Q_RBOD_BOD_S > 1.0)
		{
			Q_RBOD_BOD_S = 1.0;
		}

		// Compute total attitude error
		VMAG = sqrt(1.0 - Q_RBOD_BOD_S * Q_RBOD_BOD_S);

		// If VMAG is smaller than a tolerance, then the total attitude error is set to zero
		if (VMAG < 0.000043)
		{
			TOT_ERR = _V(0, 0, 0);
		}
		else
		{
			// Otherwise the total attitude error is computed
			double RVMAG = 1.0 / (VMAG * RAD_PER_DEG);
			TOT_ERR = -Q_RBOD_BOD_V * 2.0 * RVMAG * acos(Q_RBOD_BOD_S);
		}
	}

	void UniversalPointing::CMPLT_TIME()
	{
		// The MNVR_CMPL_TIME is calculated for the current maneuver, tracking,and rotation options.
		// The completion time computation starts when a current tracking, maneuver, or
		// rotation option is initiated and requires the current time(GMT) and the GMT/MET reference time (BASE_MET).

		// The maneuver completion time is processed only when the maneuver time flag is set.
		if (MNVR_TIME_FLAG)
		{
			double MET;

			// Calculate the current mission elapsed time
			MET = GMT - ReadCOMPOOL_SD(SCP_T_MET_REF);

			//Is the total attitude error out of tolerance?
			if (length(TOT_ERR) > ATT_TOLERANCE * pOrbitDAP->GetDAPDeadband())
			{
				// Yes
				double MNVR_TIME;

				// Compute maneuver time remaining
				MNVR_TIME = length(TOT_ERR) / pOrbitDAP->GetDAPRate();

				// Compute maneuver completion time
				MNVR_CMPL_TIME = MNVR_TIME + MET;
			}
			else
			{
				// Maneuver is considered complete

				// Reset maneuver time flag
				MNVR_TIME_FLAG = false;

				// If the maneuver time initial condition indicator is set (MNVR_TIME_IC = 1), then the maneuver completion time is updated to the current MET
				if (MNVR_TIME_IC)
				{
					MNVR_CMPL_TIME = MET;
				}
			}
			// Reset initial condition indicator
			MNVR_TIME_IC = false;
		}
	}

	void UniversalPointing::RNDZTGT()
	{
		// This task provides a desired target line-of-sight for the Ku-band antenna management function in systems management

		// RT_LOS is the rendezvous target line-of-sight expressed in orbiter body coordinates (TBD)
		// RT_LOS = QUAT_XFORM(Q_BOD_M50_S, Q_BOD_M50_V, RREL);
	}

	bool UniversalPointing::Get_MNVR_TRACK_OPTION() const
	{
		return MNVR_TRACK_OPTION;
	}

	void UniversalPointing::GetRequiredQuaternion(double& QS, VECTOR3& QV) const
	{
		QS = Q_RBOD_M50_S;
		QV = Q_RBOD_M50_V;
	}

	VECTOR3 UniversalPointing::Get_REQD_BRATE() const
	{
		return REQD_BRATE;
	}

	double UniversalPointing::Get_GMTR() const
	{
		return GMTR;
	}
}
