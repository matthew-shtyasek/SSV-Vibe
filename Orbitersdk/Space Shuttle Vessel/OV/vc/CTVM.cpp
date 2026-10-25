#include "CTVM.h"
#include "../Atlantis.h"
#include "../RMS.h"
#include "../VideoControlUnit.h"
#include "../../SSVSound.h"


namespace vc
{
	constexpr int MENUTOP_NAME_Y = static_cast<int>(IMAGE_SIZE * 0.13);// [px]
	constexpr int MENUTOP_OPTION_Y = static_cast<int>(IMAGE_SIZE * 0.155);// [px]

	constexpr int MENUTOP_X[4] = {static_cast<int>(IMAGE_SIZE * 0.12), static_cast<int>(IMAGE_SIZE * 0.23), static_cast<int>(IMAGE_SIZE * 0.34), static_cast<int>(IMAGE_SIZE * 0.45)};// [px]

	constexpr int MENU_SELECT_LINE_OFFSET_X1 = static_cast<int>(IMAGE_SIZE * 0.06);// [px]
	constexpr int MENU_SELECT_LINE_OFFSET_X2 = static_cast<int>(IMAGE_SIZE * 0.05);// [px]
	constexpr int MENU_SELECT_LINE_OFFSET_Y1 = static_cast<int>(IMAGE_SIZE * 0.01);// [px]
	constexpr int MENU_SELECT_LINE_OFFSET_Y2 = static_cast<int>(IMAGE_SIZE * 0.02);// [px]


	constexpr int TOPLINE_Y = static_cast<int>(IMAGE_SIZE * 0.17);// [px]
	constexpr int BOTTOMLINE_Y = static_cast<int>(IMAGE_SIZE * 0.77);// [px]

	constexpr int ZOOMLABEL_X = static_cast<int>(IMAGE_SIZE * 0.15);// [px]
	constexpr int PANLABEL_X = static_cast<int>(IMAGE_SIZE * 0.11);// [px]
	constexpr int TILTLABEL_X = static_cast<int>(IMAGE_SIZE * 0.33);// [px]
	constexpr int NAMELABEL_X = static_cast<int>(IMAGE_SIZE * 0.85);// [px]

	constexpr int CROSSHAIR_LENGTH = 20;// [px]

	constexpr double MENU_DISPLAY_TIME = 10.0;// time menu stays visible after last input [s]

	// OVRLY: PDRS target overlay for the grapple (on), with the distance and angle to the grapple fixture in view (hint)
	constexpr int MENU_ITEMS = 4;
	char* MENU_HEADINGS[MENU_ITEMS] = {"L-DATA", "C-DATA", "XHAIR", "OVRLY"};
	constexpr unsigned int MENU_OPTIONS_MAX_IDX[MENU_ITEMS] = {1, 2, 2, 2};
	char* MENU_OPTIONS[MENU_ITEMS][3] = {{"off", "on", ""},
				{"off", "grn", "wht"},
				{"off", "grn", "wht"},
				{"off", "on", "hint"}};

	#define CR_LIGHT_GREEN RGB( 0, 255, 0 )

	constexpr unsigned int MAX_LIGHT_VTX = 64;
	constexpr float POWER_LIGHT_ON_OFFSET_U = 0.0;
	constexpr float POWER_LIGHT_ON_OFFSET_V = 34.0 / 2048;


	static oapi::Pen* skpThickLightGreenPen = NULL;
	static oapi::Pen* skpThinLightGreenPen = NULL;
	static oapi::Font* skpFontData = NULL;
	static oapi::Font* skpFontMenu = NULL;


	CTVM::CTVM( unsigned short id, Atlantis* _sts, const string& _ident ):AtlantisVCComponent( _sts, _ident ),
		power(true), id(id), menuoptions{0,0,0}, menuselect(0), powerswitch(false), menu(false), selectpressed(false), functionleft(false), functionright(false), menutime(0.0),
		pPower(NULL), pFunction(NULL), pSelect(NULL), anim_power(NULL), anim_function(NULL), anim_select(NULL)
	{
		if (STS()->D3D9())
		{
			if (!(hSurf = oapiCreateSurfaceEx( IMAGE_SIZE, IMAGE_SIZE, OAPISURFACE_RENDER3D | OAPISURFACE_TEXTURE | OAPISURFACE_RENDERTARGET | OAPISURFACE_NOMIPMAPS ))) throw std::exception( "oapiCreateSurfaceEx() failed" );

			if (!skpThickLightGreenPen)
				if (!(skpThickLightGreenPen = oapiCreatePen( 1, 3, CR_LIGHT_GREEN ))) throw std::exception( "oapiCreatePen() failed" );
			if (!skpThinLightGreenPen)
				if (!(skpThinLightGreenPen = oapiCreatePen( 1, 1, CR_LIGHT_GREEN ))) throw std::exception( "oapiCreatePen() failed" );
			if (!skpFontData)
				if (!(skpFontData = oapiCreateFontEx( 34, "Sans", 14, FW_BOLD, FontStyle::FONT_NORMAL, 0.0f ))) throw std::exception( "oapiCreateFontEx() failed" );
			if (!skpFontMenu)
				if (!(skpFontMenu = oapiCreateFontEx( 17, "Sans", 7, FW_BOLD, FontStyle::FONT_NORMAL, 0.0f ))) throw std::exception( "oapiCreateFontEx() failed" );}
		else
		{
			hSurf = NULL;
		}
		return;
	}

	CTVM::~CTVM( void )
	{
		if (skpThickLightGreenPen)
		{
			oapiReleasePen( skpThickLightGreenPen );
			skpThickLightGreenPen = NULL;
		}
		if (skpThinLightGreenPen)
		{
			oapiReleasePen( skpThinLightGreenPen );
			skpThinLightGreenPen = NULL;
		}
		if (skpFontData)
		{
			oapiReleaseFont( skpFontData );
			skpFontData = NULL;
		}
		if (skpFontMenu)
		{
			oapiReleaseFont( skpFontMenu );
			skpFontMenu = NULL;
		}

		if (pPower) delete pPower;
		if (pFunction) delete pFunction;
		if (pSelect) delete pSelect;
		if (hSurf) oapiDestroySurface( hSurf );
		return;
	}

	void CTVM::SetReferences( const VECTOR3& refPower, const VECTOR3& refFunction )
	{
		this->refPower = refPower;
		this->refFunction = refFunction;
		return;
	}

	void CTVM::SetDirections( const VECTOR3& dirPower, const VECTOR3& dirFunction, const VECTOR3& dirSelect )
	{
		this->dirPower = dirPower;
		this->dirFunction = dirFunction;
		this->dirSelect = dirSelect;
		return;
	}

	void CTVM::SetGroups( UINT panelmesh, UINT grpPower, UINT grpFunction, UINT grpSelect, DWORD grpPowerLight, DWORD texScreen )
	{
		this->panelmesh = panelmesh;
		this->grpPower = grpPower;
		this->grpFunction = grpFunction;
		this->grpSelect = grpSelect;
		this->grpPowerLight = grpPowerLight;
		this->texScreen = texScreen;
		return;
	}

	void CTVM::ConnectPowerSource( discsignals::DiscreteBundle* pBundle, unsigned short usLine )
	{
		PowerSource.Connect( pBundle, usLine );
		return;
	}

	void CTVM::DefineVCAnimations( UINT vc_idx )
	{
		pPower = new MGROUP_ROTATE( vc_idx, &grpPower, 1, refPower, dirPower, static_cast<float>(66.0 * RAD) );
		anim_power = STS()->CreateAnimation( 0.5 );
		STS()->AddAnimationComponent( anim_power, 0.0, 1.0, pPower, NULL );
		OnPowerChange();

		pFunction = new MGROUP_ROTATE( vc_idx, &grpFunction, 1, refFunction, dirFunction, static_cast<float>(66.0 * RAD) );
		anim_function = STS()->CreateAnimation( 0.5 );
		STS()->AddAnimationComponent( anim_function, 0.0, 1.0, pFunction, NULL );

		pSelect = new MGROUP_TRANSLATE( vc_idx, &grpSelect, 1, dirSelect * 0.0025 );
		anim_select = STS()->CreateAnimation( 0.0 );
		STS()->AddAnimationComponent( anim_select, 0.0, 1.0, pSelect, NULL );
		return;
	}

	bool CTVM::OnMouseEvent( int _event, float x, float y )
	{
		constexpr float power_x_min = 0.2036f;
		constexpr float power_x_max = 0.3671f;
		constexpr float power_y_min = 0.1467f;
		constexpr float power_y_max = 0.2495f;

		constexpr float select_x_min = 0.1774f;
		constexpr float select_x_max = 0.4237f;
		constexpr float select_y_min = 0.3321f;
		constexpr float select_y_max = 0.4129f;

		constexpr float function_x_min = 0.3274f;
		constexpr float function_x_max = 0.6402f;
		constexpr float function_y_min = 0.4610f;
		constexpr float function_y_max = 0.5148f;

		// power
		if ((x > power_x_min) && (x < power_x_max))
		{
			if ((y > power_y_min) && (y < (power_y_min + 0.4 * (power_y_max - power_y_min))))
			{
				// on
				if (_event & PANEL_MOUSE_LBDOWN)
				{
					OnPowerOn();
					SoundPlay( STS()->GetSound(), SWITCH_THROW_SOUND );
					return true;
				}
			}
			else if ((y < power_y_max) && (y > (power_y_max - 0.4 * (power_y_max - power_y_min))))
			{
				// off
				if (_event & PANEL_MOUSE_LBDOWN)
				{
					OnPowerOff();
					SoundPlay( STS()->GetSound(), SWITCH_THROW_SOUND );
					return true;
				}
			}
		}
		// select
		if ((x > select_x_min) && (x < select_x_max))
		{
			if ((y > select_y_min) && (y < select_y_max))
			{
				if (_event & PANEL_MOUSE_LBDOWN)
				{
					OnSelectPress();
					SoundPlay( STS()->GetSound(), KEY_PRESS_SOUND );
					return true;
				}
				else if (_event & PANEL_MOUSE_LBUP)
				{
					OnSelectDepress();
					return true;
				}
			}
		}
		// function
		if ((y > function_y_min) && (y < function_y_max))
		{
			if ((x > function_x_min) && (x < (function_x_min + 0.4 * (function_x_max - function_x_min))))
			{
				if (_event & PANEL_MOUSE_LBDOWN)
				{
					OnFunctionLeft();
					SoundPlay( STS()->GetSound(), SWITCH_THROW_SOUND );
					return true;
				}
				else if (_event & PANEL_MOUSE_LBUP)
				{
					OnFunctionCenter();
					SoundPlay( STS()->GetSound(), SWITCH_THROW_SOUND );
					return true;
				}
			}
			else if ((x < function_x_max) && (x > (function_x_max - 0.4 * (function_x_max - function_x_min))))
			{
				if (_event & PANEL_MOUSE_LBDOWN)
				{
					OnFunctionRight();
					SoundPlay( STS()->GetSound(), SWITCH_THROW_SOUND );
					return true;
				}
				else if (_event & PANEL_MOUSE_LBUP)
				{
					OnFunctionCenter();
					SoundPlay( STS()->GetSound(), SWITCH_THROW_SOUND );
					return true;
				}
			}
		}
		return false;
	}

	void CTVM::OnPowerOn( void )
	{
		powerswitch = true;
		OnPowerChange();
		return;
	}

	void CTVM::OnPowerOff( void )
	{
		powerswitch = false;
		OnPowerChange();
		return;
	}

	void CTVM::OnPowerChange( void )
	{
		STS()->SetAnimation( anim_power, powerswitch ? 1.0 : 0.0 );
		return;
	}

	void CTVM::OnSelectPress( void )
	{
		selectpressed = true;
		STS()->SetAnimation( anim_select, 1.0 );
		return;
	}

	void CTVM::OnSelectDepress( void )
	{
		STS()->SetAnimation( anim_select, 0.0 );
		return;
	}

	void CTVM::OnFunctionLeft( void )
	{
		functionleft = true;
		STS()->SetAnimation( anim_function, 1.0 );
		return;
	}

	void CTVM::OnFunctionCenter( void )
	{
		STS()->SetAnimation( anim_function, 0.5 );
		return;
	}

	void CTVM::OnFunctionRight( void )
	{
		functionright = true;
		STS()->SetAnimation( anim_function, 0.0 );
		return;
	}

	bool CTVM::OnReadState( FILEHANDLE scn )
	{
		char* line;

		while (oapiReadScenario_nextline( scn, line ))
		{
			if (!_strnicmp( line, "@ENDOBJECT", 10 ))
			{
				return true;
			}
			else if (!_strnicmp( line, "POWER", 5 ))
			{
				if (!_strnicmp( line + 6, "ON", 2 )) powerswitch = true;
			}
			else if (!_strnicmp( line, "L-DATA", 6 ))
			{
				sscanf_s( (char*)(line + 7), "%hu", &menuoptions[0] );
				if (menuoptions[0] > MENU_OPTIONS_MAX_IDX[0]) menuoptions[0] = MENU_OPTIONS_MAX_IDX[0];
			}
			else if (!_strnicmp( line, "C-DATA", 6 ))
			{
				sscanf_s( (char*)(line + 7), "%hu", &menuoptions[1] );
				if (menuoptions[1] > MENU_OPTIONS_MAX_IDX[1]) menuoptions[1] = MENU_OPTIONS_MAX_IDX[1];
			}
			else if (!_strnicmp( line, "XHAIR", 5 ))
			{
				sscanf_s( (char*)(line + 6), "%hu", &menuoptions[2] );
				if (menuoptions[2] > MENU_OPTIONS_MAX_IDX[2]) menuoptions[2] = MENU_OPTIONS_MAX_IDX[2];
			}
			else if (!_strnicmp( line, "OVRLY", 5 ))
			{
				sscanf_s( (char*)(line + 6), "%hu", &menuoptions[3] );
				if (menuoptions[3] > MENU_OPTIONS_MAX_IDX[3]) menuoptions[3] = MENU_OPTIONS_MAX_IDX[3];
			}
		}
		return false;
	}

	void CTVM::OnSaveState( FILEHANDLE scn ) const
	{
		oapiWriteScenario_string( scn, "POWER", powerswitch ? "ON" : "OFF" );
		oapiWriteScenario_int( scn, "L-DATA", menuoptions[0] );
		oapiWriteScenario_int( scn, "C-DATA", menuoptions[1] );
		oapiWriteScenario_int( scn, "XHAIR", menuoptions[2] );
		oapiWriteScenario_int( scn, "OVRLY", menuoptions[3] );
		return;
	}

	void CTVM::OnPreStep( double simt, double simdt, double mjd )
	{
		// only works in D3D9
		if (!STS()->D3D9()) return;

		bool new_power = powerswitch && PowerSource.IsSet();
		if (new_power != power)
		{
			power = new_power;

			selectpressed = false;
			functionleft = false;
			functionright = false;
			menu = false;
			menuselect = 0;
			menuoptions[0] = 0;
			menuoptions[1] = 0;
			menuoptions[2] = 0;
			menuoptions[3] = 0;

			oapiClearSurface( hSurf );

			UpdateLightUV();
		}
		if (!power) return;

		// handle menu visibility and config
		if (selectpressed || functionleft || functionright)
		{
			// set timer of menu off
			menutime = simt + MENU_DISPLAY_TIME;

			// accept inputs if menu already on
			if (menu)
			{
				if (selectpressed)
				{
					if (menuoptions[menuselect] == MENU_OPTIONS_MAX_IDX[menuselect]) menuoptions[menuselect] = 0;
					else menuoptions[menuselect]++;
				}
				else if (functionleft)
				{
					if (menuselect == 0) menuselect = MENU_ITEMS - 1;
					else menuselect--;
				}
				else if (functionright)
				{
					if (menuselect == (MENU_ITEMS - 1)) menuselect = 0;
					else menuselect++;
				}
			}
			else
			{
				// show menu
				menu = true;
			}
			selectpressed = false;
			functionleft = false;
			functionright = false;
		}
		else if (menu)
		{
			if (menutime <= simt)
			{
				menu = false;
			}
		}

		// draw on image
		std::string name = "";
		double pan = 0.0;
		double tilt = 0.0;
		double zoom = 0.0;
		char cbuf[32];

		// get image
		STS()->GetVCU()->GetMonitorImage( id, hSurf, name, pan, tilt, zoom );

		oapi::Sketchpad* skp = oapiGetSketchpad( hSurf );
		if (skp == NULL)
		{
			oapiWriteLog( "(SSV_OV) [ERROR] CTVM skp" );
			return;
		}

		skp->SetTextColor( CR_LIGHT_GREEN );

		//// menu ////
		if (menu)
		{
			skp->SetTextAlign( oapi::Sketchpad::TAlign_horizontal::CENTER );
			skp->SetFont( skpFontMenu );
			skp->SetPen( skpThinLightGreenPen );
			for (int i = 0; i < MENU_ITEMS; i++)
			{
				skp->Text( MENUTOP_X[i], MENUTOP_NAME_Y, MENU_HEADINGS[i], strlen( MENU_HEADINGS[i] ) );
				skp->Text( MENUTOP_X[i], MENUTOP_OPTION_Y, MENU_OPTIONS[i][menuoptions[i]], strlen( MENU_OPTIONS[i][menuoptions[i]] ) );
			}

			skp->Line( MENUTOP_X[menuselect] + MENU_SELECT_LINE_OFFSET_X1, MENUTOP_NAME_Y + MENU_SELECT_LINE_OFFSET_Y1, MENUTOP_X[menuselect] + MENU_SELECT_LINE_OFFSET_X2, MENUTOP_NAME_Y + MENU_SELECT_LINE_OFFSET_Y1 );
			skp->Line( MENUTOP_X[menuselect] - MENU_SELECT_LINE_OFFSET_X1, MENUTOP_NAME_Y + MENU_SELECT_LINE_OFFSET_Y1, MENUTOP_X[menuselect] - MENU_SELECT_LINE_OFFSET_X2, MENUTOP_NAME_Y + MENU_SELECT_LINE_OFFSET_Y1 );
			skp->Line( MENUTOP_X[menuselect] + MENU_SELECT_LINE_OFFSET_X1, MENUTOP_NAME_Y + MENU_SELECT_LINE_OFFSET_Y1, MENUTOP_X[menuselect] + MENU_SELECT_LINE_OFFSET_X1, MENUTOP_NAME_Y + MENU_SELECT_LINE_OFFSET_Y2 );
			skp->Line( MENUTOP_X[menuselect] - MENU_SELECT_LINE_OFFSET_X1, MENUTOP_NAME_Y + MENU_SELECT_LINE_OFFSET_Y1, MENUTOP_X[menuselect] - MENU_SELECT_LINE_OFFSET_X1, MENUTOP_NAME_Y + MENU_SELECT_LINE_OFFSET_Y2 );
		}

		skp->SetTextAlign( oapi::Sketchpad::TAlign_horizontal::LEFT );
		skp->SetFont( skpFontData );
		skp->SetPen( skpThickLightGreenPen );

		//// lens data ////
		if (menuoptions[0] == 1)
		{
			// zoom
			sprintf_s( cbuf, 32, "%04.1f ºFOV", zoom );
			skp->Text( ZOOMLABEL_X, TOPLINE_Y, cbuf, strlen( cbuf ) );
		}

		//// camera data ////
		if (menuoptions[1] == 1)
		{
			// pan
			sprintf_s( cbuf, 32, "%+06.1fP", pan );
			skp->Text( PANLABEL_X, BOTTOMLINE_Y, cbuf, strlen( cbuf ) );
			// tilt
			sprintf_s( cbuf, 32, "%+06.1fT", tilt );
			skp->Text( TILTLABEL_X, BOTTOMLINE_Y, cbuf, strlen( cbuf ) );
			// name
			skp->Text( NAMELABEL_X, BOTTOMLINE_Y, name.c_str(), name.length() );
		}

		//// crosshairs ////
		if (menuoptions[2] == 1)
		{
			skp->Line( IMAGE_SIZE2, IMAGE_SIZE2 - CROSSHAIR_LENGTH, IMAGE_SIZE2, IMAGE_SIZE2 + CROSSHAIR_LENGTH );// center V
			skp->Line( IMAGE_SIZE2 - CROSSHAIR_LENGTH, IMAGE_SIZE2, IMAGE_SIZE2 + CROSSHAIR_LENGTH, IMAGE_SIZE2 );// center H
			skp->Line( 0, IMAGE_SIZE2, CROSSHAIR_LENGTH, IMAGE_SIZE2 );// left
			skp->Line( IMAGE_SIZE - CROSSHAIR_LENGTH, IMAGE_SIZE2, IMAGE_SIZE, IMAGE_SIZE2 );// right
			skp->Line( IMAGE_SIZE2, IMAGE_SIZE / 8, IMAGE_SIZE2, (IMAGE_SIZE / 8) + CROSSHAIR_LENGTH );// top
			skp->Line( IMAGE_SIZE2, IMAGE_SIZE - (IMAGE_SIZE / 8) - CROSSHAIR_LENGTH, IMAGE_SIZE2, IMAGE_SIZE );// bottom
		}

		//// PDRS target overlay: the transparent overlay that is put over the monitor to grapple with the wrist camera (34 deg FOV), and an optional training hint ////
		const int overlay = menuoptions[3];
		if ((overlay > 0) && (name == "RMS") && (pan == 0.0) && (tilt == 0.0))
		{
			const int cx = IMAGE_SIZE2;
			const int cy = IMAGE_SIZE2;
			const double radii[2] = {IMAGE_SIZE * 0.10, IMAGE_SIZE * 0.20};
			for (int k = 0; k < 2; k++)
			{
				const int segments = 48;
				int px = cx + static_cast<int>(radii[k]);
				int py = cy;
				for (int i = 1; i <= segments; i++)
				{
					double a = (2.0 * 3.14159265358979 * i) / segments;
					int x = cx + static_cast<int>(radii[k] * cos( a ));
					int y = cy + static_cast<int>(radii[k] * sin( a ));
					skp->Line( px, py, x, y );
					px = x;
					py = y;
				}
			}
			// alignment marks of the outer circle and the center cross
			for (int i = 0; i < 4; i++)
			{
				const int dx = (i == 0) - (i == 1);
				const int dy = (i == 2) - (i == 3);
				skp->Line( cx + static_cast<int>(dx * radii[1]), cy + static_cast<int>(dy * radii[1]), cx + static_cast<int>(dx * (radii[1] + 14)), cy + static_cast<int>(dy * (radii[1] + 14)) );
			}
			skp->Line( cx - 6, cy, cx + 6, cy );
			skp->Line( cx, cy - 6, cx, cy + 6 );

			if (overlay > 1)
			{
				RMS* pRMS = STS()->GetRMS();
				double d, a;
				if ((pRMS != NULL) && pRMS->GetGrappleHint( d, a ))
				{
					sprintf_s( cbuf, 32, "GF %.2fm %.1fdeg", d, a * DEG );
					skp->Text( NAMELABEL_X - 150, TOPLINE_Y, cbuf, strlen( cbuf ) );
				}
			}
		}

		oapiReleaseSketchpad( skp );
		return;
	}

	void CTVM::VisualCreated( void )
	{
		// only works in D3D9
		if (!STS()->D3D9()) return;

		// handle screen texture
		DEVMESHHANDLE hDevMesh = STS()->GetDevMesh( STS()->Get_vis(), panelmesh );
		if (hDevMesh != NULL) oapiSetTexture( hDevMesh, texScreen, hSurf );

		// handle power light
		UpdateLightUV();
		return;
	}

	void CTVM::UpdateLightUV( void )
	{
		if (STS()->Get_vis() == 0) return;
		MESHHANDLE panelTemplate = STS()->GetMeshTemplate( panelmesh );
		MESHGROUPEX* mg = oapiMeshGroupEx( panelTemplate, grpPowerLight );
		DEVMESHHANDLE hDevpanelmesh = STS()->GetDevMesh( STS()->Get_vis(), panelmesh );

		assert( (mg->nVtx <= MAX_LIGHT_VTX) && "CTVM::UpdateLightUV.mg->nVtx" );
		static NTVERTEX Vtx[MAX_LIGHT_VTX];
		for (unsigned short i = 0; i < mg->nVtx; i++)
		{
			Vtx[i].tu = mg->Vtx[i].tu + (power ? POWER_LIGHT_ON_OFFSET_U : 0.0f);
			Vtx[i].tv = mg->Vtx[i].tv + (power ? POWER_LIGHT_ON_OFFSET_V : 0.0f);
		}

		GROUPEDITSPEC grpSpec;
		grpSpec.flags = GRPEDIT_VTXTEX;
		grpSpec.Vtx = Vtx;
		grpSpec.nVtx = mg->nVtx;
		grpSpec.vIdx = NULL;

		oapiEditMeshGroup( hDevpanelmesh, grpPowerLight, &grpSpec );
		return;
	}
};
