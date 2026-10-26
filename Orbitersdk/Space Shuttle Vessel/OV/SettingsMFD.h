/****************************************************************************
  This file is part of Space Shuttle Vessel

  Settings MFD: shows the options of the vessel and changes them without editing the files. The changes are applied at once, and the SAVE
  button writes them to Config\SSV\Settings.cfg (read at the start of the next simulation).


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
#ifndef _SETTINGSMFD_H_
#define _SETTINGSMFD_H_


#include <Orbitersdk.h>
#include "SettingsModel.h"


class Atlantis;

class SettingsMFD: public MFD2
{
public:
	SettingsMFD( DWORD w, DWORD h, VESSEL* vessel );
	~SettingsMFD();

	char* ButtonLabel( int bt ) override;
	int ButtonMenu( const MFDBUTTONMENU** menu ) const override;
	bool ConsumeKeyBuffered( DWORD key ) override;
	bool ConsumeButton( int bt, int event ) override;
	bool Update( oapi::Sketchpad* skp ) override;
	static int MsgProc( UINT msg, UINT mfd, WPARAM wparam, LPARAM lparam );

private:
	void Apply( void );// the values of the editor to the options of the vessel
	void Save( void );
	void DrawSettings( oapi::Sketchpad* skp );
	void DrawFailures( oapi::Sketchpad* skp );
	void DrawStatus( oapi::Sketchpad* skp );
	bool KeyFailures( DWORD key );

	Atlantis* sts;
	settings::Editor editor;
	int page;
	int failRow;// selected line of the page of the failures
	char status[48];
	double statusTime;
};

#endif// _SETTINGSMFD_H_
