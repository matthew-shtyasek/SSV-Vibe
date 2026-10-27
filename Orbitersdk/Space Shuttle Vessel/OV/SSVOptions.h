/****************************************************************************
  This file is part of Space Shuttle Vessel

  SSV Options definition


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
2020/05/01   GLS
2020/06/20   GLS
2020/06/21   GLS
2021/07/03   GLS
2021/08/24   GLS
2022/08/05   GLS
2022/10/06   GLS
2022/10/07   GLS
2023/02/08   GLS
2026/02/22   GLS
********************************************/
#ifndef SSV_OPTIONS
#define SSV_OPTIONS


#include <OrbiterAPI.h>
#include <string>
#include "ParameterValues.h"
#include "SettingsModel.h"


/**
 * Options of the vessel. The values come from Config\Vessels\SSV_OV.cfg, and then from the file of the user settings (Config\SSV\Settings.cfg, that
 * the settings MFD writes), which has the same format and only needs the options that differ. The table of the options is in SettingsModel.h.
 */
class SSVOptions
{
	private:
		double values[settings::COUNT];

		static constexpr const char* USER_FILE = "SSV\Settings.cfg";// in the folder Config

		void ReadItems( FILEHANDLE cfg )
		{
			for (int i = 0; i < settings::COUNT; i++)
			{
				const settings::Item& it = settings::ITEMS[i];
				if (it.kind == settings::KIND_BOOL)
				{
					bool b = (values[i] != 0.0);
					if (oapiReadItem_bool( cfg, const_cast<char*>(it.key), b )) values[i] = b ? 1.0 : 0.0;
				}
				else if ((it.kind == settings::KIND_INT) || (it.kind == settings::KIND_CHOICE))
				{
					int n = (int)values[i];
					if (oapiReadItem_int( cfg, const_cast<char*>(it.key), n )) values[i] = settings::Clamp( i, n );
				}
				else
				{
					double d = values[i];
					if (oapiReadItem_float( cfg, const_cast<char*>(it.key), d )) values[i] = settings::Clamp( i, d );
				}
			}
		}

	public:
		SSVOptions(void)
		{
			for (int i = 0; i < settings::COUNT; i++) values[i] = settings::ITEMS[i].def;
		}

		virtual ~SSVOptions( void )
		{
		}

		/** Reads the options of the configuration file of the vessel class. */
		void Parse( FILEHANDLE cfg )
		{
			ReadItems( cfg );
			return;
		}

		/** Reads the settings of the user (the options that are there replace the ones of the class). */
		void LoadUser( void )
		{
			FILEHANDLE f = oapiOpenFile( USER_FILE, FILE_IN_ZEROONFAIL, CONFIG );
			if (f == 0) return;
			ReadItems( f );
			oapiCloseFile( f, FILE_IN_ZEROONFAIL );
			return;
		}

		/**
		 * Writes the settings of the user.
		 * @return false if the file could not be written
		 */
		bool SaveUser( void ) const
		{
			FILEHANDLE f = oapiOpenFile( USER_FILE, FILE_OUT, CONFIG );
			if (f == 0) return false;
			oapiWriteLine( f, "; User settings of the vessel SSV_OV (written by the settings MFD). The options of Config\Vessels\SSV_OV.cfg are replaced by these." );
			for (int i = 0; i < settings::COUNT; i++)
			{
				const settings::Item& it = settings::ITEMS[i];
				if (it.kind == settings::KIND_BOOL) oapiWriteItem_bool( f, const_cast<char*>(it.key), values[i] != 0.0 );
				else if ((it.kind == settings::KIND_INT) || (it.kind == settings::KIND_CHOICE)) oapiWriteItem_int( f, const_cast<char*>(it.key), (int)values[i] );
				else oapiWriteItem_float( f, const_cast<char*>(it.key), values[i] );
			}
			oapiCloseFile( f, FILE_OUT );
			return true;
		}

		/** @return the value of an option (a boolean is 0 or 1) */
		double Get( int i ) const
		{
			return values[i];
		}

		/** Sets the value of an option (limited to its range). */
		void Set( int i, double v )
		{
			values[i] = settings::Clamp( i, v );
		}

		bool EIUDataRecorder( void ) const
		{
			return values[settings::EIU_DATA_RECORDER] != 0.0;
		}

		unsigned short PositionLabelTime( void ) const
		{
			return static_cast<unsigned short>(values[settings::POSITION_LABEL_TIME]);
		}

		bool AutoActionLandingGear( void ) const
		{
			return values[settings::AUTO_ACTION_LANDING_GEAR] != 0.0;
		}

		bool AutoActionDragChute( void ) const
		{
			return values[settings::AUTO_ACTION_DRAG_CHUTE] != 0.0;
		}

		double RHCRate( void ) const
		{
			return values[settings::ITEM_RHC_RATE];
		}

		double RPTARate( void ) const
		{
			return values[settings::ITEM_RPTA_RATE];
		}

		double BrakeRate( void ) const
		{
			return values[settings::ITEM_BRAKE_RATE];
		}

		bool Alarm1x( void ) const
		{
			return values[settings::ALARM_1X] != 0.0;
		}

		bool RMSPayloadMassEffect( void ) const
		{
			return values[settings::RMS_PAYLOAD_MASS_EFFECT] != 0.0;
		}

		bool RMSReaction( void ) const
		{
			return values[settings::RMS_REACTION] != 0.0;
		}

		bool RMSCenterOfMass( void ) const
		{
			return values[settings::RMS_CENTER_OF_MASS] != 0.0;
		}

		bool RMSInertia( void ) const
		{
			return values[settings::RMS_INERTIA] != 0.0;
		}

		bool RMSViaGPC( void ) const
		{
			return values[settings::RMS_VIA_GPC] != 0.0;
		}

		bool CabinLighting( void ) const
		{
			return values[settings::CABIN_LIGHTING] != 0.0;
		}

		bool VCFreeMove( void ) const
		{
			return values[settings::VC_FREE_MOVE] != 0.0;
		}

		/** @return the index of the key of the free movement of the head (settings::FREE_MOVE_KEYS) */
		int VCFreeMoveKey( void ) const
		{
			return (int)values[settings::VC_FREE_MOVE_KEY];
		}

		bool HydraulicThermal( void ) const
		{
			return values[settings::HYDRAULIC_THERMAL] != 0.0;
		}

		/** @return rate of the random failures of each boiler [1/hour], 0 if off */
		double HydraulicFailureRate( void ) const
		{
			const int i = static_cast<int>(values[settings::HYDRAULIC_RANDOM_FAILURES] + 0.5);
			return ((i >= 0) && (i < settings::HYD_FAILURE_RATE_COUNT)) ? settings::HYD_FAILURE_RATES[i] : 0.0;
		}
};

#endif// SSV_OPTIONS
