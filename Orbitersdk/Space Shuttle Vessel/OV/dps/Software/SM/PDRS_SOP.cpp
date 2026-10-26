#include "PDRS_SOP.h"
#include "../../SimpleGPCSystem.h"
#include "../../../Atlantis.h"
#include "../../../RMS.h"


namespace dps
{
	PDRS_SOP::PDRS_SOP( SimpleGPCSystem* _gpc ):SimpleGPCSoftware( _gpc, "PDRS_SOP" ),
		timeSinceCycle(0.0)
	{
	}

	PDRS_SOP::~PDRS_SOP( void )
	{
		RMS* pRMS = STS()->GetRMS();
		if ((pRMS != NULL) && (pRMS->GetGpcLogic() == &logic)) pRMS->SetGpcLogic( NULL );
	}

	void PDRS_SOP::OnPostStep( double simt, double simdt, double mjd )
	{
		RMS* pRMS = STS()->GetRMS();
		if (pRMS == NULL) return;

		// the arm and the software find each other (the arm may be created after the GPC)
		if (pRMS->GetGpcLogic() != &logic) pRMS->SetGpcLogic( &logic );

		// the software runs in SM OPS 2
		if ((ReadCOMPOOL_IS( SCP_MM ) / 100) != 2)
		{
			timeSinceCycle = 0.0;
			return;
		}

		timeSinceCycle += simdt;
		if (timeSinceCycle < pdrs::GPC_CYCLE) return;
		timeSinceCycle = fmod( timeSinceCycle, pdrs::GPC_CYCLE );

		pdrs::Mailbox& mb = pRMS->GetMailbox();
		pdrs::Input in;
		pdrs::UnpackInput( mb.toGpc, in );
		mb.counterRead++;

		pdrs::Output out;
		logic.Step( in, out );
		pdrs::PackOutput( out, mb.fromGpc );
		mb.counterOut++;
	}

	bool PDRS_SOP::OnMajorModeChange( unsigned int newMajorMode )
	{
		// the software is active in SM OPS 2 (as the other SM software of the on-orbit major modes)
		switch (newMajorMode)
		{
			case 201:
			case 202:
				return true;
			default:
				return false;
		}
	}
}
