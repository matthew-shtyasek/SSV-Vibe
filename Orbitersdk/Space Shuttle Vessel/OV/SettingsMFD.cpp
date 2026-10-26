#include "SettingsMFD.h"
#include "Atlantis.h"
#include "SSVOptions.h"


namespace
{
	constexpr DWORD COLOR_TEXT = 0x00E0E0E0;// 0x00BBGGRR
	constexpr DWORD COLOR_VALUE = 0x0000FF00;
	constexpr DWORD COLOR_SELECTED = 0x0000FFFF;
	constexpr DWORD COLOR_CHANGED = 0x0000A0FF;
	constexpr DWORD COLOR_NOTE = 0x00B0B0B0;
	constexpr DWORD COLOR_WARNING = 0x004040FF;
	constexpr double STATUS_TIME = 4.0;// [s] time that a message stays

	// the buttons: left 0-5, right 6-11
	enum {BT_UP = 0, BT_DOWN, BT_GROUP_PREV, BT_GROUP_NEXT, BT_NONE4, BT_NONE5, BT_DEC, BT_INC, BT_TOGGLE, BT_SAVE, BT_DEFAULT, BT_DEFAULT_ALL};
}


SettingsMFD::SettingsMFD( DWORD w, DWORD h, VESSEL* vessel ): MFD2( w, h, vessel ), statusTime( -100.0 )
{
	sts = static_cast<Atlantis*>(vessel);
	status[0] = 0;
	double v[settings::COUNT];
	for (int i = 0; i < settings::COUNT; i++) v[i] = sts->GetOptions()->Get( i );
	editor.SetValues( v );
}

SettingsMFD::~SettingsMFD()
{
}

void SettingsMFD::Apply( void )
{
	for (int i = 0; i < settings::COUNT; i++) sts->GetOptions()->Set( i, editor.Value( i ) );
}

void SettingsMFD::Save( void )
{
	if (sts->GetOptions()->SaveUser())
	{
		editor.MarkSaved();
		sprintf_s( status, sizeof( status ), "Saved in Config\\SSV\\Settings.cfg" );
	}
	else sprintf_s( status, sizeof( status ), "The file could not be written" );
	statusTime = oapiGetSysTime();
}

bool SettingsMFD::Update( oapi::Sketchpad* skp )
{
	try
	{
		Title( skp, "SSV SETTINGS" );
		skp->SetTextColor( COLOR_TEXT );

		const int line = static_cast<int>(ch);
		int y = 3 * line;
		const int rows = static_cast<int>((H - (8 * ch)) / ch);// the title above, the note and the status below
		const int first = editor.FirstRow( rows );
		const int right = static_cast<int>(W) - static_cast<int>(2 * cw);
		char buf[64];

		for (int r = 0; r < rows; r++)
		{
			const int i = first + r;
			if (i >= settings::COUNT) break;
			const settings::Item& it = settings::ITEMS[i];
			const bool selected = (i == editor.Cursor());
			const bool changedFromDefault = !settings::IsDefault( i, editor.Value( i ) );

			skp->SetTextColor( selected ? COLOR_SELECTED : COLOR_TEXT );
			skp->Text( static_cast<int>(cw), y, selected ? ">" : " ", 1 );
			sprintf_s( buf, sizeof( buf ), "%-4s %s%s", settings::GroupName( it.group ), it.label, it.restart ? "*" : "" );
			// the group name only on the first line of the group
			if ((i > 0) && (settings::ITEMS[i - 1].group == it.group)) sprintf_s( buf, sizeof( buf ), "     %s%s", it.label, it.restart ? "*" : "" );
			skp->Text( static_cast<int>(3 * cw), y, buf, static_cast<int>(strlen( buf )) );

			char val[24];
			settings::FormatValue( i, editor.Value( i ), val, sizeof( val ) );
			skp->SetTextColor( selected ? COLOR_SELECTED : (changedFromDefault ? COLOR_CHANGED : COLOR_VALUE) );
			skp->SetTextAlign( oapi::Sketchpad::RIGHT );
			skp->Text( right, y, val, static_cast<int>(strlen( val )) );
			skp->SetTextAlign( oapi::Sketchpad::LEFT );
			y += line;
		}

		// the note of the selected option
		y = static_cast<int>(H) - static_cast<int>(5 * ch);
		skp->SetTextColor( COLOR_NOTE );
		const char* note = settings::ITEMS[editor.Cursor()].note;
		const int perLine = static_cast<int>(W / cw) - 2;
		int pos = 0;
		const int len = static_cast<int>(strlen( note ));
		for (int l = 0; (l < 3) && (pos < len); l++)
		{
			int n = len - pos;
			if (n > perLine)
			{
				n = perLine;
				while ((n > 0) && (note[pos + n] != ' ')) n--;// do not cut a word
				if (n == 0) n = perLine;
			}
			skp->Text( static_cast<int>(cw), y, note + pos, n );
			pos += n;
			while ((pos < len) && (note[pos] == ' ')) pos++;
			y += line;
		}

		// the last line: message, or the state of the file
		y = static_cast<int>(H) - static_cast<int>(2 * ch);
		if ((oapiGetSysTime() - statusTime) < STATUS_TIME)
		{
			skp->SetTextColor( COLOR_VALUE );
			skp->Text( static_cast<int>(cw), y, status, static_cast<int>(strlen( status )) );
		}
		else if (editor.IsDirty())
		{
			skp->SetTextColor( COLOR_WARNING );
			skp->Text( static_cast<int>(cw), y, "Not saved (SAVE)", 16 );
		}
		else if (settings::ITEMS[editor.Cursor()].restart)
		{
			skp->SetTextColor( COLOR_NOTE );
			skp->Text( static_cast<int>(cw), y, "* takes effect at the next start", 32 );
		}
		return true;
	}
	catch (std::exception& e)
	{
		oapiWriteLogV( "(SettingsMFD) [FATAL ERROR] Exception in Update: %s", e.what() );
		abort();
	}
	catch (...)
	{
		oapiWriteLog( "(SettingsMFD) [FATAL ERROR] Exception in Update" );
		abort();
	}
}

char* SettingsMFD::ButtonLabel( int bt )
{
	static char* labels[12] = {"UP", "DWN", "GRP-", "GRP+", 0, 0, "DEC", "INC", "TOG", "SAVE", "DEF", "ALL"};
	return (bt >= 0 && bt < 12) ? labels[bt] : 0;
}

int SettingsMFD::ButtonMenu( const MFDBUTTONMENU** menu ) const
{
	static const MFDBUTTONMENU mnu[12] = {
		{"Previous option", "(Up)", 'U'},
		{"Next option", "(Down)", 'D'},
		{"Previous group", "(PgUp)", 'G'},
		{"Next group", "(PgDn)", 'H'},
		{0, 0, 0},
		{0, 0, 0},
		{"Decrease the value", "(Left)", '-'},
		{"Increase the value", "(Right)", '+'},
		{"Toggle the option", "(Enter)", 'T'},
		{"Save in the file", "(S)", 'S'},
		{"Default of the option", "(Del)", 'F'},
		{"Defaults of all", "(Shift+Del)", 'A'}
	};
	if (menu) *menu = mnu;
	return 12;
}

bool SettingsMFD::ConsumeKeyBuffered( DWORD key )
{
	bool changed = false;
	switch (key)
	{
		case OAPI_KEY_UP:
		case OAPI_KEY_U:
			editor.Move( -1 );
			break;
		case OAPI_KEY_DOWN:
		case OAPI_KEY_D:
			editor.Move( 1 );
			break;
		case OAPI_KEY_PRIOR:
		case OAPI_KEY_G:
			editor.MoveGroup( -1 );
			break;
		case OAPI_KEY_NEXT:
		case OAPI_KEY_H:
			editor.MoveGroup( 1 );
			break;
		case OAPI_KEY_LEFT:
		case OAPI_KEY_MINUS:
			changed = editor.Change( -1 );
			break;
		case OAPI_KEY_RIGHT:
		case OAPI_KEY_EQUALS:
			changed = editor.Change( 1 );
			break;
		case OAPI_KEY_RETURN:
		case OAPI_KEY_T:
		case OAPI_KEY_SPACE:
			changed = editor.Change( 1 );
			break;
		case OAPI_KEY_S:
			Save();
			break;
		case OAPI_KEY_DELETE:
		case OAPI_KEY_F:
			changed = editor.ResetItem();
			break;
		case OAPI_KEY_A:
			changed = editor.ResetAll();
			break;
		default:
			return false;
	}
	if (changed) Apply();
	InvalidateDisplay();
	return true;
}

bool SettingsMFD::ConsumeButton( int bt, int event )
{
	if (!(event & PANEL_MOUSE_LBDOWN)) return false;
	static const DWORD btkey[12] = {OAPI_KEY_UP, OAPI_KEY_DOWN, OAPI_KEY_PRIOR, OAPI_KEY_NEXT, 0, 0, OAPI_KEY_LEFT, OAPI_KEY_RIGHT, OAPI_KEY_RETURN, OAPI_KEY_S, OAPI_KEY_DELETE, OAPI_KEY_A};
	if ((bt < 0) || (bt >= 12) || (btkey[bt] == 0)) return false;
	return ConsumeKeyBuffered( btkey[bt] );
}

int SettingsMFD::MsgProc( UINT msg, UINT mfd, WPARAM wparam, LPARAM lparam )
{
	switch (msg)
	{
		case OAPI_MSG_MFD_OPENED:
			return (int)(new SettingsMFD( LOWORD( wparam ), HIWORD( wparam ), (VESSEL*)lparam ));
	}
	return 0;
}
