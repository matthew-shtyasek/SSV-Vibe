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
	using ctrlmath::SubDeviceType;
	using ctrlmath::SUB_AXIS;
	using ctrlmath::SUB_BUTTON;
	using ctrlmath::SUB_POV;
	using ctrlmath::AX_X;
	using ctrlmath::AX_Y;
	using ctrlmath::AX_Z;
	using ctrlmath::AX_RX;
	using ctrlmath::AX_RY;
	using ctrlmath::AX_RZ;
	using ctrlmath::AX_SLIDER0;
	using ctrlmath::AX_SLIDER1;

	constexpr const char* CONFIG_PATH = ".\\Config\\SSV\\Controllers\\";

	using ctrlmath::INPUT_NAMES;
	static_assert( ctrlmath::INPUT_NAME_COUNT == HandControllers::INPUT_COUNT, "the names of the inputs" );

	struct Connection
	{
		int device;
		int type;// SubDeviceType
		int id;// axis (AX_...), button number or POV number * 2 (+1 for the vertical direction)
		bool reverse;
		double deadzone;
		double expo;
		double center;
		bool armed = true;// a stick axis is used only after it was seen at its center: an axis that rests at the end (the toe brake of a pedal) is ignored
		bool warned = false;
	};

	struct Device
	{
		LPDIRECTINPUTDEVICE8 dev;
		DIJOYSTATE2 state;
		std::string name;
		std::vector<std::string> axes;// names of the axes found, for the log of detected devices
		double axisnorm[8];// normalized raw axes (X, Y, Z, RX, RY, RZ, slider 0, slider 1)
		std::string cfgpath;// file that the configuration was read from (or will be written to), empty if none
		DIJOYSTATE2 learnState;// state at the moment of BeginLearn
		double learnAxes[8];
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
	bool raw = false;// read the controllers even if none is configured
	int learning = -1;// input that waits for an assignment
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
	return ctrlmath::ParseSubDevice( text, type, id );
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

	// a file can name its device ("DEVICE<tab>part of the name"): it is then used only for a controller with that name. Without it, a file
	// that is chosen by the number (Joystick1.cfg) is applied to whatever controller is the first one, for example to a pedal when the stick is not connected
	{
		std::string wanted;
		std::string l0;
		while (std::getline( file, l0 ))
		{
			const std::vector<std::string> c0 = ctrlmath::SplitColumns( l0 );
			if ((c0.size() >= 2) && (Lower( c0[0] ) == "device"))
			{
				wanted = c0[1];
				break;
			}
		}
		file.clear();
		file.seekg( 0 );
		if (!wanted.empty() && (Lower( name ).find( Lower( wanted ) ) == std::string::npos))
		{
			oapiWriteLogV( "(SSV_OV) [INFO] Hand controllers: %s is for the controller \"%s\", not used for \"%s\"", used.c_str(), wanted.c_str(), name.c_str() );
			return;
		}
	}
	devices[deviceindex].cfgpath = used;

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

		// the stick axes (not the brakes, the speed brake or the throttle) rest at the center
		c.armed = !((c.type == SUB_AXIS) && (input < RMS_CAPTURE) && (input != SBTC) && (input != BRAKE_LEFT) && (input != BRAKE_RIGHT));

		conns[input].push_back( c );
		count++;
	}
	oapiWriteLogV( "(SSV_OV) [INFO] Hand controllers: %d connection(s) for \"%s\" read from %s", count, name.c_str(), used.c_str() );
	return;
}

bool HandControllers::Impl::ParsePanelInput( const std::string& text, std::string& panel, std::string& component, int& action ) const
{
	// the values of the actions are the same in both enumerations
	static_assert( (ctrlmath::ACT_UP == HandControllers::PA_UP) && (ctrlmath::ACT_DOWN == HandControllers::PA_DOWN) && (ctrlmath::ACT_MID == HandControllers::PA_MID) && (ctrlmath::ACT_TOGGLE == HandControllers::PA_TOGGLE) && (ctrlmath::ACT_HOLD_UP == HandControllers::PA_HOLD_UP) && (ctrlmath::ACT_HOLD_DOWN == HandControllers::PA_HOLD_DOWN) && (ctrlmath::ACT_PRESS == HandControllers::PA_PRESS), "the actions of the panel buttons" );
	return ctrlmath::ParsePanelInput( text, panel, component, action );
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
	if (!any && !impl->raw) return;// no controller is configured

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
		std::vector<Connection>& list = impl->conns[i];
		double best = 0.0;
		bool pressed = false;
		for (auto& c : list)
		{
			double v = impl->Evaluate( c );
			if ((c.type == SUB_AXIS) && !c.armed)
			{
				// an axis of a stick that is not at its center when the simulation starts is not a stick (or is of another controller): ignored until it is
				if (fabs( v ) < 0.5) c.armed = true;
				else
				{
					if (!c.warned)
					{
						c.warned = true;
						oapiWriteLogV( "(SSV_OV) [WARNING] Hand controllers: %s on Joystick%d (%s) is not at its center (%.2f): ignored until it is. Is the configuration of this controller?", INPUT_NAMES[i], c.device + 1, impl->devices[c.device].name.c_str(), v );
					}
					v = 0.0;
				}
			}
			if (fabs( v ) > fabs( best )) best = v;
			if (v > 0.7) pressed = true;
		}
		impl->axisvalue[i] = best;
		impl->buttonvalue[i] = pressed;
	}

	if ((impl->learning >= 0) && LearnStep( impl->learning )) impl->learning = -1;
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


const char* HandControllers::InputName( int input )
{
	return ((input >= 0) && (input < INPUT_COUNT)) ? INPUT_NAMES[input] : "?";
}

int HandControllers::DeviceCount( void ) const
{
	return (int)impl->devices.size();
}

std::string HandControllers::DeviceName( int index ) const
{
	if ((index < 0) || (index >= (int)impl->devices.size())) return "";
	return impl->devices[index].name;
}

void HandControllers::SetRawPolling( bool on )
{
	impl->raw = on;
}

std::string HandControllers::DescribeInput( int input ) const
{
	if ((input < 0) || (input >= INPUT_COUNT) || impl->conns[input].empty()) return "-";
	std::string t;
	for (const auto& c : impl->conns[input])
	{
		if (!t.empty()) t += ", ";
		t += "J" + std::to_string( c.device + 1 ) + " " + ctrlmath::FormatSubDevice( c.type, c.id );
		if (c.reverse) t += " (rev)";
	}
	return t;
}

void HandControllers::BeginLearn( void )
{
	for (auto& d : impl->devices)
	{
		d.learnState = d.state;
		for (int k = 0; k < 8; k++) d.learnAxes[k] = d.axisnorm[k];
	}
}

bool HandControllers::LearnStep( int input )
{
	if ((input < 0) || (input >= INPUT_COUNT) || !impl->di) return false;

	int dev = -1;
	int type = SUB_AXIS;
	int id = 0;
	bool reverse = false;

	for (size_t i = 0; (i < impl->devices.size()) && (dev < 0); i++)
	{
		const Device& d = impl->devices[i];
		if (IsAxisInput( input ))
		{
			const int a = ctrlmath::DetectAxisMove( d.learnAxes, d.axisnorm, 8, 0.5 );
			if (a >= 0)
			{
				dev = (int)i;
				type = SUB_AXIS;
				id = a;
			}
		}
		else
		{
			for (int k = 0; k < 128; k++)
			{
				if (((d.state.rgbButtons[k] & 0x80) != 0) && ((d.learnState.rgbButtons[k] & 0x80) == 0))
				{
					dev = (int)i;
					type = SUB_BUTTON;
					id = k;
					break;
				}
			}
			for (int p = 0; (p < 4) && (dev < 0); p++)
			{
				double h, v;
				double h0, v0;
				if (!ctrlmath::PovToDirection( d.state.rgdwPOV[p], h, v )) continue;
				if (ctrlmath::PovToDirection( d.learnState.rgdwPOV[p], h0, v0 )) continue;// was not at rest
				dev = (int)i;
				type = SUB_POV;
				if (fabs( h ) > 0.5)
				{
					id = p * 2;
					reverse = (h < 0.0);
				}
				else
				{
					id = (p * 2) + 1;
					reverse = (v < 0.0);
				}
			}
		}
	}
	if (dev < 0) return false;

	Connection c;
	c.device = dev;
	c.type = type;
	c.id = id;
	c.reverse = reverse;
	c.deadzone = (type == SUB_AXIS) ? ctrlmath::DefaultDeadzone( INPUT_NAMES[input] ) : 0.0;
	c.expo = 0.0;
	c.center = 0.0;
	impl->conns[input].clear();
	impl->conns[input].push_back( c );
	oapiWriteLogV( "(SSV_OV) [INFO] Hand controllers: %s assigned to Joystick%d %s", INPUT_NAMES[input], dev + 1, ctrlmath::FormatSubDevice( type, id ).c_str() );
	return true;
}

void HandControllers::StartLearn( int input )
{
	if ((input < 0) || (input >= INPUT_COUNT)) return;
	BeginLearn();
	impl->learning = input;
}

void HandControllers::StopLearn( void )
{
	impl->learning = -1;
}

int HandControllers::LearningInput( void ) const
{
	return impl->learning;
}

void HandControllers::ClearInput( int input )
{
	if ((input < 0) || (input >= INPUT_COUNT)) return;
	impl->conns[input].clear();
	impl->axisvalue[input] = 0.0;
	impl->buttonvalue[input] = false;
}

bool HandControllers::ToggleReverse( int input )
{
	if ((input < 0) || (input >= INPUT_COUNT) || impl->conns[input].empty()) return false;
	for (auto& c : impl->conns[input]) c.reverse = !c.reverse;
	return true;
}

bool HandControllers::SaveConfigs( std::string& message )
{
	int written = 0;
	bool ok = true;
	for (size_t i = 0; i < impl->devices.size(); i++)
	{
		Device& d = impl->devices[i];
		std::vector<std::string> lines;
		for (int input = 0; input < INPUT_COUNT; input++)
		{
			for (const auto& c : impl->conns[input])
			{
				if (c.device != (int)i) continue;
				lines.push_back( ctrlmath::FormatConfigLine( INPUT_NAMES[input], c.type, c.id, c.reverse, c.deadzone, c.expo, c.center ) );
			}
		}
		for (const auto& b : impl->panelbindings)
		{
			for (const auto& c : b.conns)
			{
				if (c.device != (int)i) continue;
				lines.push_back( ctrlmath::FormatConfigLine( "PANEL " + b.panel + "/" + b.component + ":" + ctrlmath::ActionName( b.action ), c.type, c.id, c.reverse, 0.0, 0.0, 0.0 ) );
			}
		}
		if (lines.empty() && d.cfgpath.empty()) continue;

		const std::string path = d.cfgpath.empty() ? (std::string( CONFIG_PATH ) + "Joystick" + std::to_string( i + 1 ) + ".cfg") : d.cfgpath;
		if (!d.cfgpath.empty()) CopyFileA( path.c_str(), (path + ".bak").c_str(), TRUE );// the first version stays as .bak

		std::ofstream file( path );
		if (!file.is_open())
		{
			ok = false;
			continue;
		}
		file << "# Written by the controllers page of the SSV settings MFD for: " << d.name << "\n";
		file << "# The file is used only for a controller whose name contains the text of the line DEVICE\n";
		file << "DEVICE\t" << d.name << "\n";
		file << "# INPUT\tSUBDEVID\tREVERSE\tDEADZONE\tEXPO\tCENTER\n";
		for (const auto& l : lines) file << l << "\n";
		d.cfgpath = path;
		written++;
	}
	message = ok ? ("Saved " + std::to_string( written ) + " file(s) in Config\\SSV\\Controllers") : "A file could not be written";
	return ok;
}
