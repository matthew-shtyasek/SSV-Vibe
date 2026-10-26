/****************************************************************************
  This file is part of Space Shuttle Vessel

  PDRS software of the SM GPC: the computer supported modes of the Remote Manipulator System


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
#ifndef _dps_PDRS_SOP_H_
#define _dps_PDRS_SOP_H_


#include "../SimpleGPCSoftware.h"
#include "../../../PDRSLogic.h"


namespace dps
{
	/**
	 * @brief	Software of the PDRS (Payload Deployment and Retrieval System) that runs in the SM GPC.
	 *
	 * It runs at 12.5 Hz in the SM OPS 2 major modes (MM 2xx). In each cycle it reads the frame that the MCIU of the arm has put on the bus (hand
	 * controllers, switches of the D&C panel, angles of the joints), runs the modes of the arm (see PDRSLogic.h) and returns the commands of rate
	 * of the joints. The displays SPEC 94, DISP 95, DISP 96 and DISP 169 show and change the data of the logic.
	 * Without the software (other major modes, GPC not running it) the arm can only be driven with DIRECT and BACKUP, as in the real vehicle.
	 */
	class PDRS_SOP:public SimpleGPCSoftware
	{
		private:
			pdrs::Logic logic;
			double timeSinceCycle;

		public:
			explicit PDRS_SOP( SimpleGPCSystem* _gpc );
			~PDRS_SOP( void );

			void OnPostStep( double simt, double simdt, double mjd ) override;
	};
}


#endif// _dps_PDRS_SOP_H_
