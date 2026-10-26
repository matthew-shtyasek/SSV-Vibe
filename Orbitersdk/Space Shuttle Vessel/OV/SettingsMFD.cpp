#include "SettingsMFD.h"
#include "Atlantis.h"
#include "SSVOptions.h"
#include "WSB.h"


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


SettingsMFD::SettingsMFD( DWORD w, DWORD h, VESSEL* vessel ): MFD2( w, h, vessel ), statusTime( -100.0 ), page( settings::PAGE_SETTINGS ), failRow( 0 )
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
		if (page == settings::PAGE_FAILURES) DrawFailures( skp );
		else if (page == settings::PAGE_STATUS) DrawStatus( skp );
		else DrawSettings( skp );
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

void SettingsMFD::DrawFailures( oapi::Sketchpad* skp )
{
	Title( skp, "SSV HYDRAULIC FAILURES" );
	const int line = static_cast<int>(ch);
	const int right = static_cast<int>(W) - static_cast<int>(2 * cw);
	const int rows = static_cast<int>((H - (7 * ch)) / ch);
	int first = failRow - (rows / 2);
	if (first > (settings::FAIL_ROWS - rows)) first = settings::FAIL_ROWS - rows;
	if (first < 0) first = 0;

	static const char* const kinds[settings::FAIL_KINDS] = {"controller A", "controller B", "ice, hydraulic spray", "ice, lube oil spray", "N2 leak", "hydraulic leak"};
	int y = 3 * line;
	char buf[64];
	for (int r = 0; (r < rows) && ((first + r) < settings::FAIL_ROWS); r++)
	{
		const int row = first + r;
		const int sys = settings::FailSystem( row );
		const int kind = settings::FailKind( row );
		WSB* wsb = sts->GetWSB( sys );
		const bool active = (wsb != NULL) && wsb->GetFailure( kind );
		const bool selected = (row == failRow);
		skp->SetTextColor( selected ? COLOR_SELECTED : COLOR_TEXT );
		skp->Text( static_cast<int>(cw), y, selected ? ">" : " ", 1 );
		sprintf_s( buf, sizeof( buf ), "WSB %d  %s", sys, kinds[kind] );
		skp->Text( static_cast<int>(3 * cw), y, buf, static_cast<int>(strlen( buf )) );
		skp->SetTextColor( selected ? COLOR_SELECTED : (active ? COLOR_WARNING : COLOR_VALUE) );
		skp->SetTextAlign( oapi::Sketchpad::RIGHT );
		const char* val = active ? "FAILED" : "OK";
		skp->Text( right, y, val, static_cast<int>(strlen( val )) );
		skp->SetTextAlign( oapi::Sketchpad::LEFT );
		y += line;
	}
	y = static_cast<int>(H) - static_cast<int>(3 * ch);
	skp->SetTextColor( COLOR_NOTE );
	skp->Text( static_cast<int>(cw), y, "Failures of the boilers, to train", 33 );
	skp->Text( static_cast<int>(cw), y + line, "TOG: fail   DEF: repair   ALL: repair all", 41 );
}

void SettingsMFD::DrawStatus( oapi::Sketchpad* skp )
{
	Title( skp, "SSV HYDRAULIC STATUS" );
	const int line = static_cast<int>(ch);
	int y = 3 * line;
	char buf[80];
	for (int sys = 1; sys <= settings::FAIL_SYSTEMS; sys++)
	{
		WSB* wsb = sts->GetWSB( sys );
		skp->SetTextColor( COLOR_TEXT );
		sprintf_s( buf, sizeof( buf ), "SYSTEM %d", sys );
		skp->Text( static_cast<int>(cw), y, buf, static_cast<int>(strlen( buf )) );
		y += line;
		if (wsb == NULL)
		{
			skp->Text( static_cast<int>(2 * cw), y, "no data", 7 );
			y += 2 * line;
			continue;
		}
		const double oil = wsb->GetLubeOilTemperature();
		const double hyd = wsb->GetHydraulicFluidTemperature();
		skp->SetTextColor( (oil > hydmodel::OIL_LIMIT) ? COLOR_WARNING : COLOR_VALUE );
		sprintf_s( buf, sizeof( buf ), "  OIL %4.0f F", oil );
		skp->Text( static_cast<int>(cw), y, buf, static_cast<int>(strlen( buf )) );
		skp->SetTextColor( (hyd > hydmodel::HYD_CAUTION) ? COLOR_WARNING : COLOR_VALUE );
		sprintf_s( buf, sizeof( buf ), "HYD %4.0f F", hyd );
		skp->Text( static_cast<int>(19 * cw), y, buf, static_cast<int>(strlen( buf )) );
		y += line;
		skp->SetTextColor( COLOR_VALUE );
		sprintf_s( buf, sizeof( buf ), "  H2O %3.0f LB   RSVR %3.0f%%", wsb->GetWater(), 100.0 * wsb->GetHydraulicQuantity() );
		skp->Text( static_cast<int>(cw), y, buf, static_cast<int>(strlen( buf )) );
		y += line;
		if (wsb->BearingsSeized())
		{
			skp->SetTextColor( COLOR_WARNING );
			skp->Text( static_cast<int>(3 * cw), y, "APU BEARINGS SEIZED", 19 );
		}
		y += line;
	}
}

void SettingsMFD::DrawSettings( oapi::Sketchpad* skp )
{
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
	}
}

char* SettingsMFD::ButtonLabel( int bt )
{
	static char* labels[12] = {"UP", "DWN", "GRP-", "GRP+", "PAGE", 0, "DEC", "INC", "TOG", "SAVE", "DEF", "ALL"};
	return (bt >= 0 && bt < 12) ? labels[bt] : 0;
}

int SettingsMFD::ButtonMenu( const MFDBUTTONMENU** menu ) const
{
	static const MFDBUTTONMENU mnu[12] = {
		{"Previous option", "(Up)", 'U'},
		{"Next option", "(Down)", 'D'},
		{"Previous group", "(PgUp)", 'G'},
		{"Next group", "(PgDn)", 'H'},
		{"Next page", "(P)", 'P'},
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

bool SettingsMFD::KeyFailures( DWORD key )
{
	WSB* wsb = sts->GetWSB( settings::FailSystem( failRow ) );
	const int kind = settings::FailKind( failRow );
	switch (key)
	{
		case OAPI_KEY_UP:
		case OAPI_KEY_U:
			failRow = (failRow + settings::FAIL_ROWS - 1) % settings::FAIL_ROWS;
			break;
		case OAPI_KEY_DOWN:
		case OAPI_KEY_D:
			failRow = (failRow + 1) % settings::FAIL_ROWS;
			break;
		case OAPI_KEY_PRIOR:
		case OAPI_KEY_G:
			failRow = (((failRow / settings::FAIL_KINDS) + settings::FAIL_SYSTEMS - 1) % settings::FAIL_SYSTEMS) * settings::FAIL_KINDS;
			break;
		case OAPI_KEY_NEXT:
		case OAPI_KEY_H:
			failRow = (((failRow / settings::FAIL_KINDS) + 1) % settings::FAIL_SYSTEMS) * settings::FAIL_KINDS;
			break;
		case OAPI_KEY_LEFT:
		case OAPI_KEY_MINUS:
		case OAPI_KEY_RIGHT:
		case OAPI_KEY_EQUALS:
		case OAPI_KEY_RETURN:
		case OAPI_KEY_T:
		case OAPI_KEY_SPACE:
			if (wsb != NULL) wsb->SetFailure( kind, !wsb->GetFailure( kind ) );
			break;
		case OAPI_KEY_DELETE:
		case OAPI_KEY_F:
			if (wsb != NULL) wsb->SetFailure( kind, false );
			break;
		case OAPI_KEY_A:
			for (int s = 1; s <= settings::FAIL_SYSTEMS; s++)
			{
				WSB* w = sts->GetWSB( s );
				if (w == NULL) continue;
				for (int k = 0; k < settings::FAIL_KINDS; k++) w->SetFailure( k, false );
			}
			break;
		default:
			return false;
	}
	return true;
}

bool SettingsMFD::ConsumeKeyBuffered( DWORD key )
{
	if (key == OAPI_KEY_P)
	{
		page = settings::NextPage( page, 1 );
		InvalidateDisplay();
		return true;
	}
	if (page == settings::PAGE_STATUS) return false;
	if (page == settings::PAGE_FAILURES)
	{
		if (KeyFailures( key ))
		{
			InvalidateDisplay();
			return true;
		}
		return false;
	}

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
	static const DWORD btkey[12] = {OAPI_KEY_UP, OAPI_KEY_DOWN, OAPI_KEY_PRIOR, OAPI_KEY_NEXT, OAPI_KEY_P, 0, OAPI_KEY_LEFT, OAPI_KEY_RIGHT, OAPI_KEY_RETURN, OAPI_KEY_S, OAPI_KEY_DELETE, OAPI_KEY_A};
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
