/****************************************************************************
  This file is part of Space Shuttle Vessel

  Hand controller input (RHC and THC) from game controllers


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
#define NOMINMAX
#define DIRECTINPUT_VERSION 0x0800
#include "HandControllers.h"
#include "ControllerMath.h"
#include <dinput.h>
#include <OrbiterAPI.h>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>


#pragma comment( lib, "dinput8.lib" )
#pragma comment( lib, "dxguid.lib" )


namespace
{
	enum SubDeviceType {SUB_AXIS = 0, SUB_BUTTON, SUB_POV};

	// axis identifiers of DIJOYSTATE2
	enum {AX_X = 0, AX_Y, AX_Z, AX_RX, AX_RY, AX_RZ, AX_SLIDER0, AX_SLIDER1};

	constexpr const char* CONFIG_PATH = ".\\Config\\SSV\\Controllers\\";

	const char* const INPUT_NAMES[HandControllers::INPUT_COUNT] = {
		"RHC_PITCH", "RHC_YAW", "RHC_ROLL", "THC_X", "THC_Y", "THC_Z", "RPTA", "SBTC", "BRAKE_LEFT", "BRAKE_RIGHT",
		"RMS_CAPTURE", "RMS_RELEASE", "RMS_RATE_HOLD", "RMS_RATE_TOGGLE", "RMS_CONTROL_TOGGLE"
	};

	struct Connection
	{
		int device;
		int type;// SubDeviceType
		int id;// axis (AX_...), button number or POV number * 2 (+1 for the vertical direction)
		bool reverse;
		double deadzone;
		double expo;
		double center;
	};

	struct Device
	{
		LPDIRECTINPUTDEVICE8 dev;
		DIJOYSTATE2 state;
		std::string name;
		std::vector<std::string> axes;// names of the axes found, for the log of detected devices
		double axisnorm[8];// normalized raw axes (X, Y, Z, RX, RY, RZ, slider 0, slider 1)
	};

	BOOL CALLBACK EnumDevicesCallback( const DIDEVICEINSTANCE* inst, VOID* ctx );
	BOOL CALLBACK EnumAxesCallback( const DIDEVICEOBJECTINSTANCE* obj, VOID* ctx );

	std::string Trim( const std::string& s )
	{
		size_t a = s.find_first_not_of( " \t\r\n" );
		if (a == std::string::npos) return "";
		size_t b = s.find_last_not_of( " \t\r\n" );
		return s.substr( a, b - a + 1 );
	}

	std::string Lower( std::string s )
	{
		std::transform( s.begin(), s.end(), s.begin(), []( unsigned char c ) { return (char)std::tolower( c ); } );
		return s;
	}

	std::string FileSafe( std::string s )
	{
		for (auto& c : s)
		{
			if ((c == '\\') || (c == '/') || (c == ':') || (c == '*') || (c == '?') || (c == '"') || (c == '<') || (c == '>') || (c == '|')) c = '_';
		}
		return Trim( s );
	}

	double Clamp( double v ) { return std::max( -1.0, std::min( 1.0, v ) ); }
}


struct HandControllers::Impl
{
	LPDIRECTINPUT8 di = NULL;
	bool initialized = false;
	std::vector<Device> devices;
	std::vector<Connection> conns[INPUT_COUNT];

	struct PanelBinding
	{
		std::string panel;
		std::string component;
		int action;
		std::vector<Connection> conns;
		bool pressed;
	};
	std::vector<PanelBinding> panelbindings;
	std::vector<PanelEvent> events;
	bool ParsePanelInput( const std::string& text, std::string& panel, std::string& component, int& action ) const;
	PanelBinding* FindBinding( const std::string& panel, const std::string& component, int action );
	double axisvalue[INPUT_COUNT] = {0.0};
	bool buttonvalue[INPUT_COUNT] = {false};

	void LoadConfig( int deviceindex );
	bool ParseSubDevice( const std::string& text, int& type, int& id ) const;
	double Evaluate( const Connection& c ) const;
	void WriteDetectedDevices( void ) const;
};


namespace
{
	BOOL CALLBACK EnumDevicesCallback( const DIDEVICEINSTANCE* inst, VOID* ctx )
	{
		HandControllers::Impl* impl = static_cast<HandControllers::Impl*>(ctx);
		LPDIRECTINPUTDEVICE8 dev = NULL;
		if (FAILED( impl->di->CreateDevice( inst->guidInstance, &dev, NULL ) )) return DIENUM_CONTINUE;

		Device d;
		d.dev = dev;
		d.name = FileSafe( inst->tszProductName );
		ZeroMemory( &d.state, sizeof( d.state ) );
		for (int k = 0; k < 8; k++) d.axisnorm[k] = 0.0;
		impl->devices.push_back( d );
		return DIENUM_CONTINUE;
	}

	BOOL CALLBACK EnumAxesCallback( const DIDEVICEOBJECTINSTANCE* obj, VOID* ctx )
	{
		Device* d = static_cast<Device*>(ctx);

		// range -1000 to +1000 with no dead zone (it is applied later, with the configured values)
		DIPROPRANGE range;
		range.diph.dwSize = sizeof( DIPROPRANGE );
		range.diph.dwHeaderSize = sizeof( DIPROPHEADER );
		range.diph.dwHow = DIPH_BYID;
		range.diph.dwObj = obj->dwType;
		range.lMin = -1000;
		range.lMax = 1000;
		d->dev->SetProperty( DIPROP_RANGE, &range.diph );

		DIPROPDWORD deadzone;
		deadzone.diph.dwSize = sizeof( DIPROPDWORD );
		deadzone.diph.dwHeaderSize = sizeof( DIPROPHEADER );
		deadzone.diph.dwHow = DIPH_BYID;
		deadzone.diph.dwObj = obj->dwType;
		deadzone.dwData = 0;
		d->dev->SetProperty( DIPROP_DEADZONE, &deadzone.diph );

		if (IsEqualGUID( obj->guidType, GUID_XAxis )) d->axes.push_back( "Axis X" );
		else if (IsEqualGUID( obj->guidType, GUID_YAxis )) d->axes.push_back( "Axis Y" );
		else if (IsEqualGUID( obj->guidType, GUID_ZAxis )) d->axes.push_back( "Axis Z" );
		else if (IsEqualGUID( obj->guidType, GUID_RxAxis )) d->axes.push_back( "Axis RX" );
		else if (IsEqualGUID( obj->guidType, GUID_RyAxis )) d->axes.push_back( "Axis RY" );
		else if (IsEqualGUID( obj->guidType, GUID_RzAxis )) d->axes.push_back( "Axis RZ" );
		else if (IsEqualGUID( obj->guidType, GUID_Slider )) d->axes.push_back( "Slider" );
		return DIENUM_CONTINUE;
	}
}


bool HandControllers::Impl::ParseSubDevice( const std::string& text, int& type, int& id ) const
{
	std::string t = Lower( Trim( text ) );
	if (t.compare( 0, 4, "axis" ) == 0)
	{
		std::string a = Trim( t.substr( 4 ) );
		type = SUB_AXIS;
		if (a == "x") id = AX_X;
		else if (a == "y") id = AX_Y;
		else if (a == "z") id = AX_Z;
		else if (a == "rx") id = AX_RX;
		else if (a == "ry") id = AX_RY;
		else if (a == "rz") id = AX_RZ;
		else return false;
		return true;
	}
	else if (t.compare( 0, 6, "slider" ) == 0)
	{
		std::string a = Trim( t.substr( 6 ) );
		type = SUB_AXIS;
		if (a == "0") id = AX_SLIDER0;
		else if (a == "1") id = AX_SLIDER1;
		else return false;
		return true;
	}
	else if (t.compare( 0, 6, "button" ) == 0)
	{
		int n = atoi( Trim( t.substr( 6 ) ).c_str() );
		if ((n < 0) || (n > 127)) return false;
		type = SUB_BUTTON;
		id = n;
		return true;
	}
	else if (t.compare( 0, 3, "pov" ) == 0)
	{
		// "POV 0H": horizontal (+1 = right), "POV 0V": vertical (+1 = down)
		std::string a = Trim( t.substr( 3 ) );
		if (a.length() != 2) return false;
		int n = a[0] - '0';
		if ((n < 0) || (n > 3)) return false;
		if (a[1] == 'h') id = n * 2;
		else if (a[1] == 'v') id = (n * 2) + 1;
		else return false;
		type = SUB_POV;
		return true;
	}
	return false;
}

void HandControllers::Impl::LoadConfig( int deviceindex )
{
	// the configuration is looked for by the name of the controller, and then by its number (Joystick1.cfg, Joystick2.cfg, ...)
	std::string name = devices[deviceindex].name;
	std::string candidates[2];
	candidates[0] = std::string( CONFIG_PATH ) + name + ".cfg";
	candidates[1] = std::string( CONFIG_PATH ) + "Joystick" + std::to_string( deviceindex + 1 ) + ".cfg";

	std::ifstream file;
	std::string used;
	for (int i = 0; i < 2; i++)
	{
		file.open( candidates[i] );
		if (file.is_open())
		{
			used = candidates[i];
			break;
		}
	}
	if (!file.is_open()) return;

	int count = 0;
	std::string line;
	while (std::getline( file, line ))
	{
		// columns separated by tabs or commas: INPUT, SUBDEVID, REVERSE, DEADZONE, EXPO, CENTER; "#" starts a comment
		const std::vector<std::string> col = ctrlmath::SplitColumns( line );
		if (col.size() < 2) continue;

		std::string inputname = col[0];
		std::transform( inputname.begin(), inputname.end(), inputname.begin(), []( unsigned char c ) { return (char)std::toupper( c ); } );
		if (inputname == "INPUT") continue;// header

		// a button that operates a switch or pushbutton of a panel
		if (inputname.compare( 0, 6, "PANEL " ) == 0)
		{
			std::string pnl, comp;
			int act;
			Connection pc;
			pc.device = deviceindex;
			if (ParsePanelInput( Trim( col[0] ), pnl, comp, act ) && ParseSubDevice( col[1], pc.type, pc.id ))
			{
				pc.reverse = (col.size() > 2) && (Lower( col[2] ) == "true");
				pc.deadzone = 0.0;
				pc.expo = 0.0;
				pc.center = 0.0;
				PanelBinding* b = FindBinding( pnl, comp, act );
				if (b == NULL)
				{
					PanelBinding nb;
					nb.panel = pnl;
					nb.component = comp;
					nb.action = act;
					nb.pressed = false;
					panelbindings.push_back( nb );
					b = &panelbindings.back();
				}
				b->conns.push_back( pc );
				count++;
			}
			continue;
		}

		int input = -1;
		for (int i = 0; i < HandControllers::INPUT_COUNT; i++)
		{
			if (inputname == INPUT_NAMES[i])
			{
				input = i;
				break;
			}
		}
		if (input < 0) continue;

		Connection c;
		c.device = deviceindex;
		if (!ParseSubDevice( col[1], c.type, c.id )) continue;
		c.reverse = (col.size() > 2) && (Lower( col[2] ) == "true");
		c.deadzone = (col.size() > 3) && !col[3].empty() ? atof( col[3].c_str() ) : 0.05;
		c.expo = (col.size() > 4) && !col[4].empty() ? atof( col[4].c_str() ) : 0.0;
		c.center = (col.size() > 5) && !col[5].empty() ? atof( col[5].c_str() ) : 0.0;
		c.deadzone = std::max( 0.0, std::min( 0.9, c.deadzone ) );
		c.expo = std::max( 0.0, std::min( 1.0, c.expo ) );
		c.center = std::max( -1.0, std::min( 1.0, c.center ) );

		conns[input].push_back( c );
		count++;
	}
	oapiWriteLogV( "(SSV_OV) [INFO] Hand controllers: %d connection(s) for \"%s\" read from %s", count, name.c_str(), used.c_str() );
	return;
}

bool HandControllers::Impl::ParsePanelInput( const std::string& text, std::string& panel, std::string& component, int& action ) const
{
	// "PANEL A8A1/BRAKES:UP"
	size_t sp = text.find( ' ' );
	if (sp == std::string::npos) return false;
	std::string rest = Trim( text.substr( sp + 1 ) );
	size_t slash = rest.find( '/' );
	size_t colon = rest.rfind( ':' );
	if ((slash == std::string::npos) || (colon == std::string::npos) || (colon < slash)) return false;
	panel = Trim( rest.substr( 0, slash ) );
	component = Trim( rest.substr( slash + 1, colon - slash - 1 ) );
	std::string a = Lower( Trim( rest.substr( colon + 1 ) ) );
	if (a == "up") action = HandControllers::PA_UP;
	else if (a == "down") action = HandControllers::PA_DOWN;
	else if (a == "mid") action = HandControllers::PA_MID;
	else if (a == "toggle") action = HandControllers::PA_TOGGLE;
	else if (a == "hold_up") action = HandControllers::PA_HOLD_UP;
	else if (a == "hold_down") action = HandControllers::PA_HOLD_DOWN;
	else if (a == "press") action = HandControllers::PA_PRESS;
	else return false;
	return !panel.empty() && !component.empty();
}

HandControllers::Impl::PanelBinding* HandControllers::Impl::FindBinding( const std::string& panel, const std::string& component, int action )
{
	for (auto& b : panelbindings)
	{
		if ((b.action == action) && (Lower( b.panel ) == Lower( panel )) && (Lower( b.component ) == Lower( component ))) return &b;
	}
	return NULL;
}

double HandControllers::Impl::Evaluate( const Connection& c ) const
{
	const DIJOYSTATE2& st = devices[c.device].state;
	double v = 0.0;

	if (c.type == SUB_BUTTON)
	{
		v = (st.rgbButtons[c.id] & 0x80) ? 1.0 : 0.0;
		return c.reverse ? -v : v;
	}

	if (c.type == SUB_POV)
	{
		double h, vert;
		if (!ctrlmath::PovToDirection( st.rgdwPOV[c.id >> 1], h, vert )) return 0.0;// centered
		v = (c.id & 1) ? vert : h;
		return c.reverse ? -v : v;
	}

	// axis
	switch (c.id)
	{
		case AX_X: v = st.lX; break;
		case AX_Y: v = st.lY; break;
		case AX_Z: v = st.lZ; break;
		case AX_RX: v = st.lRx; break;
		case AX_RY: v = st.lRy; break;
		case AX_RZ: v = st.lRz; break;
		case AX_SLIDER0: v = st.rglSlider[0]; break;
		case AX_SLIDER1: v = st.rglSlider[1]; break;
		default: return 0.0;
	}
	// center, direction, dead zone around the center (detent of the hand controller) and exponential curve
	return ctrlmath::ShapeAxis( v, c.center, c.reverse, c.deadzone, c.expo );
}

void HandControllers::Impl::WriteDetectedDevices( void ) const
{
	std::ofstream file( std::string( CONFIG_PATH ) + "DetectedDevices.txt" );
	if (!file.is_open()) return;

	file << "Game controllers found by SSV (written when the simulation starts)\n";
	file << "To use a controller, copy one of the .cfg.example files of this folder to \"<name>.cfg\" (name as listed below) or to \"Joystick<number>.cfg\",\n";
	file << "and edit it. See README.txt.\n\n";
	for (size_t i = 0; i < devices.size(); i++)
	{
		DIDEVCAPS caps;
		caps.dwSize = sizeof( DIDEVCAPS );
		devices[i].dev->GetCapabilities( &caps );
		file << "Joystick" << (i + 1) << ": " << devices[i].name << "\n";
		file << "    axes: " << caps.dwAxes << " (";
		for (size_t k = 0; k < devices[i].axes.size(); k++) file << (k ? ", " : "") << devices[i].axes[k];
		file << "), buttons: " << caps.dwButtons << ", POV hats: " << caps.dwPOVs << "\n";
	}
	if (devices.empty()) file << "No game controllers found.\n";
	return;
}


HandControllers::HandControllers( void ):impl( new Impl() )
{
}

HandControllers::~HandControllers( void )
{
	for (auto& d : impl->devices)
	{
		if (d.dev)
		{
			d.dev->Unacquire();
			d.dev->Release();
		}
	}
	if (impl->di) impl->di->Release();
	delete impl;
}

void HandControllers::Init( void )
{
	if (impl->initialized) return;
	impl->initialized = true;

	if (FAILED( DirectInput8Create( oapiGetOrbiterInstance(), DIRECTINPUT_VERSION, IID_IDirectInput8, (void**)&impl->di, NULL ) ))
	{
		oapiWriteLog( "(SSV_OV) [WARNING] Hand controllers: DirectInput could not be initialized" );
		impl->di = NULL;
		return;
	}

	impl->di->EnumDevices( DI8DEVCLASS_GAMECTRL, EnumDevicesCallback, impl, DIEDFL_ATTACHEDONLY );

	HWND hwnd = GetActiveWindow();
	if (hwnd == NULL) hwnd = GetForegroundWindow();
	for (size_t i = 0; i < impl->devices.size(); i++)
	{
		Device& d = impl->devices[i];
		d.dev->SetDataFormat( &c_dfDIJoystick2 );
		if (FAILED( d.dev->SetCooperativeLevel( hwnd, DISCL_NONEXCLUSIVE | DISCL_FOREGROUND ) )) d.dev->SetCooperativeLevel( GetDesktopWindow(), DISCL_NONEXCLUSIVE | DISCL_BACKGROUND );
		d.dev->EnumObjects( EnumAxesCallback, &d, DIDFT_AXIS );
		d.dev->Acquire();
		oapiWriteLogV( "(SSV_OV) [INFO] Hand controllers: Joystick%d = \"%s\"", (int)(i + 1), d.name.c_str() );
		impl->LoadConfig( (int)i );
	}
	impl->WriteDetectedDevices();
	return;
}

void HandControllers::Poll( void )
{
	if (!impl->di || impl->devices.empty()) return;

	bool any = !impl->panelbindings.empty();
	for (int i = 0; i < INPUT_COUNT; i++) if (!impl->conns[i].empty()) any = true;
	if (!any) return;// no controller is configured

	for (auto& d : impl->devices)
	{
		HRESULT hr = d.dev->Poll();
		if (FAILED( hr ))
		{
			hr = d.dev->Acquire();
			if (SUCCEEDED( hr )) hr = d.dev->Poll();
		}
		if (FAILED( d.dev->GetDeviceState( sizeof( DIJOYSTATE2 ), &d.state ) ))
		{
			// not available (for example, the window has no focus): centered controller
			ZeroMemory( &d.state, sizeof( d.state ) );
			for (int k = 0; k < 4; k++) d.state.rgdwPOV[k] = 0xFFFFFFFF;
		}
	}

	for (auto& d : impl->devices)
	{
		d.axisnorm[AX_X] = Clamp( d.state.lX / 1000.0 );
		d.axisnorm[AX_Y] = Clamp( d.state.lY / 1000.0 );
		d.axisnorm[AX_Z] = Clamp( d.state.lZ / 1000.0 );
		d.axisnorm[AX_RX] = Clamp( d.state.lRx / 1000.0 );
		d.axisnorm[AX_RY] = Clamp( d.state.lRy / 1000.0 );
		d.axisnorm[AX_RZ] = Clamp( d.state.lRz / 1000.0 );
		d.axisnorm[AX_SLIDER0] = Clamp( d.state.rglSlider[0] / 1000.0 );
		d.axisnorm[AX_SLIDER1] = Clamp( d.state.rglSlider[1] / 1000.0 );
	}

	// buttons connected to the panels: the changes are queued
	for (auto& b : impl->panelbindings)
	{
		bool pressed = false;
		for (const auto& c : b.conns)
		{
			if (impl->Evaluate( c ) > 0.7) pressed = true;
		}
		if (pressed != b.pressed)
		{
			b.pressed = pressed;
			PanelEvent e;
			e.panel = b.panel;
			e.component = b.component;
			e.action = b.action;
			e.pressed = pressed;
			impl->events.push_back( e );
		}
	}

	for (int i = 0; i < INPUT_COUNT; i++)
	{
		const std::vector<Connection>& list = impl->conns[i];
		double best = 0.0;
		bool pressed = false;
		for (const auto& c : list)
		{
			double v = impl->Evaluate( c );
			if (fabs( v ) > fabs( best )) best = v;
			if (v > 0.7) pressed = true;
		}
		impl->axisvalue[i] = best;
		impl->buttonvalue[i] = pressed;
	}
	return;
}

bool HandControllers::HasInput( int input ) const
{
	if ((input < 0) || (input >= INPUT_COUNT)) return false;
	return !impl->conns[input].empty();
}

double HandControllers::GetAxis( int input ) const
{
	if ((input < 0) || (input >= INPUT_COUNT)) return 0.0;
	return impl->axisvalue[input];
}

bool HandControllers::GetButton( int input ) const
{
	if ((input < 0) || (input >= INPUT_COUNT)) return false;
	return impl->buttonvalue[input];
}

bool HandControllers::PopPanelEvent( PanelEvent& e )
{
	if (impl->events.empty()) return false;
	e = impl->events.front();
	impl->events.erase( impl->events.begin() );
	return true;
}

void HandControllers::CalibrateCenters( void )
{
	for (int i = 0; i < INPUT_COUNT; i++)
	{
		for (auto& c : impl->conns[i])
		{
			if (c.type != SUB_AXIS) continue;
			double raw = impl->devices[c.device].axisnorm[c.id];
			c.center = Clamp( raw );
			oapiWriteLogV( "(SSV_OV) [INFO] Hand controllers: center of %s on Joystick%d (%s) = %.3f, use it in the CENTER column", INPUT_NAMES[i], c.device + 1, impl->devices[c.device].name.c_str(), c.center );
		}
	}
	return;
}

std::string HandControllers::GetStatusText( void ) const
{
	static const char* const axisnames[8] = {"X", "Y", "Z", "RX", "RY", "RZ", "S0", "S1"};
	char buf[256];
	std::string t = "SSV game controllers (Ctrl+J: close, Ctrl+Shift+J: take the current position of the axes as center)\n";
	if (impl->devices.empty()) t += "no game controllers found\n";
	for (size_t i = 0; i < impl->devices.size(); i++)
	{
		const Device& d = impl->devices[i];
		sprintf_s( buf, 256, "Joystick%d: %s\n  ", (int)(i + 1), d.name.c_str() );
		t += buf;
		for (int k = 0; k < 8; k++)
		{
			sprintf_s( buf, 256, "%s %+.2f  ", axisnames[k], d.axisnorm[k] );
			t += buf;
		}
		t += "\n  buttons:";
		for (int k = 0; k < 128; k++) if (d.state.rgbButtons[k] & 0x80) t += " " + std::to_string( k );
		t += "\n";
	}
	t += "inputs:\n";
	for (int i = 0; i < INPUT_COUNT; i++)
	{
		if (impl->conns[i].empty()) continue;
		if (i < RMS_CAPTURE) sprintf_s( buf, 256, "  %s = %+.2f\n", INPUT_NAMES[i], impl->axisvalue[i] );
		else sprintf_s( buf, 256, "  %s = %s\n", INPUT_NAMES[i], impl->buttonvalue[i] ? "ON" : "off" );
		t += buf;
	}
	for (const auto& b : impl->panelbindings)
	{
		sprintf_s( buf, 256, "  PANEL %s/%s = %s\n", b.panel.c_str(), b.component.c_str(), b.pressed ? "ON" : "off" );
		t += buf;
	}
	return t;
}
