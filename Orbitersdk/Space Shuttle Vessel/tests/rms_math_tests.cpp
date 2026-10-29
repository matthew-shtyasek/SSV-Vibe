// Tests of the math of the RMS (OV/RMSMath.h and the helpers of SSVlib it uses). No Orbiter is needed to run them.
// Build and run with tests\run_tests.ps1.
#include <OrbiterAPI.h>
#include <cstdio>
#include <cstdlib>

// the SSVlib math is compiled in this file; the Orbiter calls it makes for debug output are removed
static char g_debugstring[1024];
#define oapiDebugString() g_debugstring
#define oapiWriteLog(x) ((void)0)
#include "../SSVlib/src/MathSSV.cpp"
#include "../OV/RMSMath.h"
#include "../OV/PTGMath.h"
#include "../OV/ControllerMath.h"
#include "../OV/RCSMath.h"
#include "../OV/ADIMath.h"
#include "../OV/APUModel.h"
#include "../OV/RMSReaction.h"
#include "../OV/RMSKin.h"
#include "../OV/PDRSLogic.h"
#include "../OV/PDRSProtocol.h"
#include "../OV/HydModel.h"
#include "../OV/Dispersion.h"
#include "../OV/HydActuator.h"
#include "../OV/RCSJets.h"
#include "../OV/ParameterValues.h"
#include "../OV/SettingsModel.h"
#include "../OV/FreeMoveMath.h"
#include "../OV/LightingMath.h"
#include "../OV/CabinLightingData.h"
#include "../OV/MassProps.h"
#include "../OV/HandControllers.h"
#include "../OV/FlightLog.h"
#include <fstream>
#include <map>
#include <tuple>
#include <string>
#include <vector>

static bool g_verbose = false;
static int g_checks = 0;
static int g_failed = 0;

#define CHECK( cond ) do { g_checks++; if (!(cond)) { g_failed++; printf( "FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond ); } } while (0)
#define CHECK_NEAR( a, b, tol ) do { g_checks++; if (fabs( (a) - (b) ) > (tol)) { g_failed++; printf( "FAILED %s:%d: %s = %.9f, %s = %.9f (tolerance %g)\n", __FILE__, __LINE__, #a, (double)(a), #b, (double)(b), (double)(tol) ); } } while (0)

static double frand( double a, double b )
{
	return a + ((b - a) * (rand() / (double)RAND_MAX));
}

static VECTOR3 randvec( double a, double b )
{
	return _V( frand( a, b ), frand( a, b ), frand( a, b ) );
}

static MATRIX3 randrot( void )
{
	VECTOR3 axis = randvec( -1.0, 1.0 );
	axis = axis / length( axis );
	return RMSMath::MatrixFromAxisAngle( axis, frand( 0.0, 3.0 ) );
}

static void near_vec( const VECTOR3& a, const VECTOR3& b, double tol, int line )
{
	g_checks++;
	if (length( a - b ) > tol)
	{
		g_failed++;
		printf( "FAILED line %d: (%.6f %.6f %.6f) != (%.6f %.6f %.6f)\n", line, a.x, a.y, a.z, b.x, b.y, b.z );
	}
}
#define CHECK_VEC( a, b, tol ) near_vec( (a), (b), (tol), __LINE__ )

// the IK frame is the body frame rolled by the rollout angle: the position conversion used by the RMS code must agree with it
static void test_frames( void )
{
	const double rollout = 19.48;
	for (int i = 0; i < 100; i++)
	{
		VECTOR3 v = randvec( -10.0, 10.0 );
		// RMS code: IK = Convert( RotateVectorZ( local, -rollout ) ), the controls use IK = RotateVectorX( body, rollout )
		VECTOR3 a = RotateVectorX( RMSMath::LocalToBody( v ), rollout );
		VECTOR3 b = RMSMath::LocalToBody( RotateVectorZ( v, -rollout ) );
		CHECK_VEC( a, b, 1e-9 );
	}
	// body axes to BACS: forward = -tail, starboard = -port, down = down
	CHECK_VEC( RMSMath::BodyToBACS( _V( 1.0, 2.0, 3.0 ) ), _V( -1.0, -2.0, 3.0 ), 1e-12 );
}

static void test_euler( void )
{
	for (double pitch = -80.0; pitch <= 80.0; pitch += 20.0)
	{
		for (double yaw = -60.0; yaw <= 60.0; yaw += 20.0)
		{
			for (double roll = -170.0; roll <= 170.0; roll += 20.0)
			{
				MATRIX3 m = GetRotationMatrixYZX( _V( roll, pitch, yaw ) * RAD );
				VECTOR3 a = GetYZX_PYRAnglesFromMatrix( m );
				CHECK_NEAR( a.data[PITCH] * DEG, pitch, 1e-6 );
				CHECK_NEAR( a.data[YAW] * DEG, yaw, 1e-6 );
				CHECK_NEAR( a.data[ROLL] * DEG, roll, 1e-6 );
			}
		}
	}
}

static void test_axis_angle( void )
{
	for (int i = 0; i < 200; i++)
	{
		VECTOR3 axis = randvec( -1.0, 1.0 );
		axis = axis / length( axis );
		double angle = frand( 0.05, 3.0 );
		MATRIX3 m = RMSMath::MatrixFromAxisAngle( axis, angle );

		// the matrix is a rotation, and it rotates the vectors in the right-handed direction
		VECTOR3 x = _V( 1.0, 0.0, 0.0 );
		CHECK_NEAR( length( mul( m, x ) ), 1.0, 1e-12 );
		CHECK_NEAR( dotp( mul( m, axis ), axis ), 1.0, 1e-12 );

		VECTOR3 axis2;
		double angle2;
		RMSMath::AxisAngleFromMatrix( m, axis2, angle2 );
		CHECK_NEAR( angle2, angle, 1e-9 );
		CHECK_VEC( axis2, axis, 1e-7 );
	}
	// 90 degrees about Z takes X to Y
	VECTOR3 r = mul( RMSMath::MatrixFromAxisAngle( _V( 0.0, 0.0, 1.0 ), PI / 2.0 ), _V( 1.0, 0.0, 0.0 ) );
	CHECK_VEC( r, _V( 0.0, 1.0, 0.0 ), 1e-12 );
}

// the error to the target, applied to the current state, gives the target
static void test_por_error( void )
{
	for (int i = 0; i < 200; i++)
	{
		MATRIX3 curM = randrot();
		VECTOR3 cur = randvec( -1500.0, 1500.0 );
		VECTOR3 porOffset = (i & 1) ? randvec( -3.0, 3.0 ) : _V( 0.0, 0.0, 0.0 );

		double tgtpos[3] = {frand( -1500.0, 1500.0 ), frand( -1500.0, 1500.0 ), frand( -1500.0, 1500.0 )};
		double tgtatt[3] = {frand( 0.0, 359.0 ), frand( 0.0, 80.0 ), frand( 0.0, 359.0 )};// pitch, yaw, roll

		VECTOR3 dB, axis;
		double angle;
		RMSMath::PORError( cur, curM, porOffset, tgtpos, tgtatt, dB, axis, angle );

		// attitude: the rotation of the error applied to the current attitude is the target attitude
		MATRIX3 tgtM = GetRotationMatrixYZX( _V( tgtatt[2], tgtatt[0], tgtatt[1] ) * RAD );
		MATRIX3 result = mul( RMSMath::MatrixFromAxisAngle( axis, angle ), curM );
		for (int k = 0; k < 3; k++)
		{
			VECTOR3 e = _V( (k == 0), (k == 1), (k == 2) );
			CHECK_VEC( mul( result, e ), mul( tgtM, e ), 1e-7 );
		}

		// position: moving the POR by the error puts it on the target
		VECTOR3 por = cur + (RMSMath::BodyToBACS( porOffset ) / RMSMath::IN2M_);
		VECTOR3 moved = por + (RMSMath::BodyToBACS( dB ) / RMSMath::IN2M_);
		CHECK_NEAR( moved.x, tgtpos[0], 1e-7 );
		CHECK_NEAR( moved.y, tgtpos[1], 1e-7 );
		CHECK_NEAR( moved.z, tgtpos[2], 1e-7 );
	}
}

// the rotation about the POR keeps the POR in place and rotates the EE
static void test_rotate_about_por( void )
{
	const double rollout = 19.48;
	const VECTOR3 id[3] = {_V( 1.0, 0.0, 0.0 ), _V( 0.0, 1.0, 0.0 ), _V( 0.0, 0.0, 1.0 )};
	for (int i = 0; i < 100; i++)
	{
		VECTOR3 dir = randvec( -1.0, 1.0 );
		dir = dir / length( dir );
		VECTOR3 rot = randvec( -1.0, 1.0 );
		rot = rot / length( rot );
		VECTOR3 pos = randvec( -8.0, 8.0 );
		VECTOR3 off = randvec( -3.0, 3.0 );
		VECTOR3 angles = randvec( -0.05, 0.05 );

		// POR in the IK frame, before and after
		VECTOR3 por0 = pos + RotateVectorX( off, rollout );

		VECTOR3 newPos = pos;
		VECTOR3 newDir, newRot;
		RMSMath::RotateAboutPOR( angles, id, true, off, rollout, dir, rot, newPos, newDir, newRot );

		VECTOR3 off2;
		RotateVector( off, angles, off2 );
		VECTOR3 por1 = newPos + RotateVectorX( off2, rollout );
		CHECK_VEC( por0, por1, 1e-9 );

		// the rotation does not change the lengths
		CHECK_NEAR( length( newDir ), 1.0, 1e-9 );
		CHECK_NEAR( length( newRot ), 1.0, 1e-9 );
		CHECK_NEAR( dotp( newDir, newRot ), dotp( dir, rot ), 1e-9 );

		// without POR the tip stays
		VECTOR3 tip = pos;
		RMSMath::RotateAboutPOR( angles, id, false, off, rollout, dir, rot, tip, newDir, newRot );
		CHECK_VEC( tip, pos, 1e-12 );
	}
}

// pointing frame and attitude of the UNIV PTG tracking
static void test_pointing( void )
{
	// the frame of the nadir with the LVLH reference is the LVLH frame itself (identity here)
	MATRIX3 f = ptg::PointingFrame( _V( 0.0, 0.0, 1.0 ), _V( 0.0, 1.0, 0.0 ) );
	CHECK_VEC( _V( f.m11, f.m21, f.m31 ), _V( 1.0, 0.0, 0.0 ), 1e-12 );
	CHECK_VEC( _V( f.m12, f.m22, f.m32 ), _V( 0.0, 1.0, 0.0 ), 1e-12 );
	CHECK_VEC( _V( f.m13, f.m23, f.m33 ), _V( 0.0, 0.0, 1.0 ), 1e-12 );

	for (int i = 0; i < 200; i++)
	{
		VECTOR3 u = randvec( -1.0, 1.0 );
		VECTOR3 yref = randvec( -1.0, 1.0 );
		if (i == 0) yref = u * 3.0;// reference along the direction
		MATRIX3 m = ptg::PointingFrame( u, yref );
		VECTOR3 x = _V( m.m11, m.m21, m.m31 );
		VECTOR3 y = _V( m.m12, m.m22, m.m32 );
		VECTOR3 z = _V( m.m13, m.m23, m.m33 );
		CHECK_NEAR( length( x ), 1.0, 1e-9 );
		CHECK_NEAR( length( y ), 1.0, 1e-9 );
		CHECK_NEAR( length( z ), 1.0, 1e-9 );
		CHECK_NEAR( dotp( x, y ), 0.0, 1e-9 );
		CHECK_NEAR( dotp( y, z ), 0.0, 1e-9 );
		CHECK_VEC( crossp( x, y ), z, 1e-9 );// right-handed
		CHECK_VEC( z, u / length( u ), 1e-9 );
		if (i != 0) CHECK( dotp( y, yref ) > -1e-9 );// nearest to the reference
	}

	// the body vector (from pitch and yaw) points at the target, whatever the roll about it (OM)
	for (int i = 0; i < 200; i++)
	{
		const double P = frand( 0.0, 359.0 );
		const double Y = frand( 0.0, 80.0 );
		const double OM = (i % 5 == 0) ? -1.0 : frand( 0.0, 359.0 );
		VECTOR3 u = randvec( -1.0, 1.0 );
		VECTOR3 yref = randvec( -1.0, 1.0 );
		MATRIX3 A = ptg::PointingAttitude( u, yref, ConvertPYOMToLVLH( P * RAD, Y * RAD, OM * RAD ) );
		// body vector in the body axes: first column of RotY( P ) * RotZ( Y )
		MATRIX3 RP, RY;
		GetRotMatrixY( P * RAD, RP );
		GetRotMatrixZ( Y * RAD, RY );
		MATRIX3 T = mul( RP, RY );
		VECTOR3 b = _V( T.m11, T.m21, T.m31 );
		CHECK_VEC( mul( A, b ), u / length( u ), 1e-7 );
		// a rotation matrix
		VECTOR3 c0 = _V( A.m11, A.m21, A.m31 );
		VECTOR3 c1 = _V( A.m12, A.m22, A.m32 );
		CHECK_NEAR( length( c0 ), 1.0, 1e-7 );
		CHECK_NEAR( dotp( c0, c1 ), 0.0, 1e-7 );
	}
}

static void test_celestial( void )
{
	CHECK_VEC( ptg::DirectionFromRADec( 0.0, 0.0 ), _V( 1.0, 0.0, 0.0 ), 1e-12 );
	CHECK_VEC( ptg::DirectionFromRADec( 90.0, 0.0 ), _V( 0.0, 1.0, 0.0 ), 1e-12 );
	CHECK_VEC( ptg::DirectionFromRADec( 123.0, 90.0 ), _V( 0.0, 0.0, 1.0 ), 1e-12 );
	for (int i = 0; i < 50; i++) CHECK_NEAR( length( ptg::DirectionFromRADec( frand( 0.0, 360.0 ), frand( -90.0, 90.0 ) ) ), 1.0, 1e-12 );

	// without precession and obliquity the DAP frame is the celestial frame (the two swaps of Y and Z cancel)
	const MATRIX3 I = _M( 1, 0, 0, 0, 1, 0, 0, 0, 1 );
	CHECK_VEC( ptg::CelestialToDAPFrame( _V( 0.3, 0.5, 0.8 ), I, I ), _V( 0.3, 0.5, 0.8 ), 1e-12 );
	CHECK_VEC( ptg::GlobalToDAPFrame( _V( 0.3, 0.5, 0.8 ), I ), _V( 0.3, 0.8, 0.5 ), 1e-12 );

	// the celestial pole is at the obliquity from the pole of the ecliptic (the Y axis of the Orbiter global frame, the Z axis here)
	const double eps = 23.4392911 * RAD;
	const MATRIX3 R = _M( 1, 0, 0, 0, cos( eps ), -sin( eps ), 0, sin( eps ), cos( eps ) );
	VECTOR3 pole = ptg::CelestialToDAPFrame( _V( 0.0, 0.0, 1.0 ), I, R );
	CHECK_NEAR( length( pole ), 1.0, 1e-12 );
	CHECK_NEAR( acos( pole.z ), eps, 1e-9 );

	// precession keeps the length of the vectors
	for (int i = 0; i < 50; i++)
	{
		VECTOR3 u = ptg::DirectionFromRADec( frand( 0.0, 360.0 ), frand( -90.0, 90.0 ) );
		CHECK_NEAR( length( ptg::CelestialToDAPFrame( u, M_J2000_to_M50, R ) ), 1.0, 1e-9 );
		CHECK_NEAR( length( ptg::GlobalToDAPFrame( u, M_J2000_to_M50 ) ), 1.0, 1e-9 );
	}
}

// axes, hats and lines of the configuration files of the game controllers
static void test_controller_axes( void )
{
	// full deflection, center, no dead zone
	CHECK_NEAR( ctrlmath::ShapeAxis( 1000.0, 0.0, false, 0.0, 0.0 ), 1.0, 1e-12 );
	CHECK_NEAR( ctrlmath::ShapeAxis( -1000.0, 0.0, false, 0.0, 0.0 ), -1.0, 1e-12 );
	CHECK_NEAR( ctrlmath::ShapeAxis( 0.0, 0.0, false, 0.05, 0.0 ), 0.0, 1e-12 );
	CHECK_NEAR( ctrlmath::ShapeAxis( 300.0, 0.0, false, 0.0, 0.0 ), 0.3, 1e-12 );
	// reverse
	CHECK_NEAR( ctrlmath::ShapeAxis( 300.0, 0.0, true, 0.0, 0.0 ), -0.3, 1e-12 );
	// dead zone: nothing inside, the rest of the range is rescaled to the full range
	CHECK_NEAR( ctrlmath::ShapeAxis( 40.0, 0.0, false, 0.05, 0.0 ), 0.0, 1e-12 );
	CHECK_NEAR( ctrlmath::ShapeAxis( -40.0, 0.0, false, 0.05, 0.0 ), 0.0, 1e-12 );
	CHECK_NEAR( ctrlmath::ShapeAxis( 1000.0, 0.0, false, 0.2, 0.0 ), 1.0, 1e-12 );
	CHECK_NEAR( ctrlmath::ShapeAxis( 600.0, 0.0, false, 0.2, 0.0 ), 0.5, 1e-12 );
	CHECK_NEAR( ctrlmath::ShapeAxis( -600.0, 0.0, false, 0.2, 0.0 ), -0.5, 1e-12 );
	// exponential curve: cubic at 1, the ends do not change
	CHECK_NEAR( ctrlmath::ShapeAxis( 500.0, 0.0, false, 0.0, 1.0 ), 0.125, 1e-12 );
	CHECK_NEAR( ctrlmath::ShapeAxis( 1000.0, 0.0, false, 0.0, 1.0 ), 1.0, 1e-12 );
	CHECK_NEAR( ctrlmath::ShapeAxis( 500.0, 0.0, false, 0.0, 0.5 ), 0.5 * (0.5 + 0.125), 1e-12 );
	// center of a throttle stick that does not return to zero
	CHECK_NEAR( ctrlmath::ShapeAxis( 500.0, 0.5, false, 0.0, 0.0 ), 0.0, 1e-12 );
	CHECK_NEAR( ctrlmath::ShapeAxis( 1000.0, 0.5, false, 0.0, 0.0 ), 0.5, 1e-12 );
	CHECK_NEAR( ctrlmath::ShapeAxis( -1000.0, 0.5, false, 0.0, 0.0 ), -1.0, 1e-12 );// the result stays in the range
	// monotonic and odd for any setting
	for (int i = 0; i < 100; i++)
	{
		const double dz = frand( 0.0, 0.5 );
		const double ex = frand( 0.0, 1.0 );
		const double a = frand( 0.0, 1000.0 );
		const double b = frand( a, 1000.0 );
		CHECK( ctrlmath::ShapeAxis( b, 0.0, false, dz, ex ) >= ctrlmath::ShapeAxis( a, 0.0, false, dz, ex ) );
		CHECK_NEAR( ctrlmath::ShapeAxis( -a, 0.0, false, dz, ex ), -ctrlmath::ShapeAxis( a, 0.0, false, dz, ex ), 1e-12 );
	}

	// levers and pedals that use the whole range of the axis
	CHECK_NEAR( ctrlmath::UnipolarFromAxis( -1.0 ), 0.0, 1e-12 );
	CHECK_NEAR( ctrlmath::UnipolarFromAxis( 0.0 ), 0.5, 1e-12 );
	CHECK_NEAR( ctrlmath::UnipolarFromAxis( 1.0 ), 1.0, 1e-12 );
	CHECK_NEAR( ctrlmath::UnipolarFromAxis( 3.0 ), 1.0, 1e-12 );
	CHECK_NEAR( ctrlmath::UnipolarFromAxis( -3.0 ), 0.0, 1e-12 );
}

static void test_controller_pov( void )
{
	double h, v;
	CHECK( !ctrlmath::PovToDirection( 0xFFFF, h, v ) );
	CHECK( !ctrlmath::PovToDirection( 0xFFFFFFFF, h, v ) );
	CHECK( (h == 0.0) && (v == 0.0) );
	CHECK( ctrlmath::PovToDirection( 0, h, v ) );// up
	CHECK_NEAR( h, 0.0, 1e-9 );
	CHECK_NEAR( v, -1.0, 1e-9 );
	CHECK( ctrlmath::PovToDirection( 9000, h, v ) );// right
	CHECK_NEAR( h, 1.0, 1e-9 );
	CHECK_NEAR( v, 0.0, 1e-9 );
	CHECK( ctrlmath::PovToDirection( 18000, h, v ) );// down
	CHECK_NEAR( h, 0.0, 1e-9 );
	CHECK_NEAR( v, 1.0, 1e-9 );
	CHECK( ctrlmath::PovToDirection( 27000, h, v ) );// left
	CHECK_NEAR( h, -1.0, 1e-9 );
	CHECK_NEAR( v, 0.0, 1e-9 );
	CHECK( ctrlmath::PovToDirection( 4500, h, v ) );// up and right, both at full value
	CHECK_NEAR( h, 1.0, 1e-9 );
	CHECK_NEAR( v, -1.0, 1e-9 );
	CHECK( ctrlmath::PovToDirection( 22500, h, v ) );// down and left
	CHECK_NEAR( h, -1.0, 1e-9 );
	CHECK_NEAR( v, 1.0, 1e-9 );
}

static void test_controller_config( void )
{
	std::vector<std::string> c = ctrlmath::SplitColumns( "RHC_PITCH\tAxis Y\ttrue\t0.05\t0.3\t0" );
	CHECK( c.size() == 6 );
	CHECK( (c.size() == 6) && (c[0] == "RHC_PITCH") && (c[1] == "Axis Y") && (c[2] == "true") && (c[5] == "0") );
	// commas, spaces around the columns and CRLF are accepted
	c = ctrlmath::SplitColumns( "THC_X , Axis Y , false\r" );
	CHECK( c.size() == 3 );
	CHECK( (c.size() == 3) && (c[0] == "THC_X") && (c[1] == "Axis Y") && (c[2] == "false") );
	// comments and empty lines
	CHECK( ctrlmath::SplitColumns( "# comment only" ).empty() );
	CHECK( ctrlmath::SplitColumns( "   \t  " ).empty() );
	CHECK( ctrlmath::SplitColumns( "" ).empty() );
	c = ctrlmath::SplitColumns( "RMS_CAPTURE\tButton 0\tfalse # trigger" );
	CHECK( c.size() == 3 );
	CHECK( (c.size() == 3) && (c[2] == "false") );
	// a panel button keeps the name of the component (with spaces) in the first column
	c = ctrlmath::SplitColumns( "PANEL A8A1/AUTO SEQ:TOGGLE\tButton 4" );
	CHECK( c.size() == 2 );
	CHECK( (c.size() == 2) && (c[0] == "PANEL A8A1/AUTO SEQ:TOGGLE") && (c[1] == "Button 4") );
	CHECK( ctrlmath::Trim( "  a b \r\n" ) == "a b" );
	CHECK( ctrlmath::Trim( " \t " ).empty() );
}

static void test_load_scale( void )
{
	CHECK_NEAR( RMSMath::LoadScale( 500.0, true, false ), 1.0, 1e-12 );
	CHECK_NEAR( RMSMath::LoadScale( 0.0, true, false ), 1.0, 1e-12 );
	CHECK_NEAR( RMSMath::LoadScale( 29500.0, true, false ), 0.1, 1e-12 );
	CHECK_NEAR( RMSMath::LoadScale( 100000.0, true, false ), 0.1, 1e-12 );
	CHECK_NEAR( RMSMath::LoadScale( 15000.0, true, false ), 1.0 - (0.9 * 14500.0 / 29000.0), 1e-12 );
	// the mass has no effect if the option is off, only the LOADED RATE override
	CHECK_NEAR( RMSMath::LoadScale( 29500.0, false, false ), 1.0, 1e-12 );
	CHECK_NEAR( RMSMath::LoadScale( 29500.0, false, true ), 0.5, 1e-12 );
	CHECK_NEAR( RMSMath::LoadScale( 500.0, true, true ), 0.5, 1e-12 );
	CHECK_NEAR( RMSMath::LoadScale( 29500.0, true, true ), 0.1, 1e-12 );// never below 0.1
	for (double m = 0.0; m < 40000.0; m += 1000.0)
	{
		CHECK( RMSMath::LoadScale( m, true, false ) >= RMSMath::LoadScale( m + 1000.0, true, false ) );
	}
}

static void test_safing_and_brakes( void )
{
	// who the hand controllers operate: the RMS in its station, the RCS elsewhere, Ctrl+A inverts it
	CHECK( RMSMath::ControlsRMS( true, true, false ) );
	CHECK( !RMSMath::ControlsRMS( true, false, false ) );
	CHECK( !RMSMath::ControlsRMS( true, true, true ) );
	CHECK( RMSMath::ControlsRMS( true, false, true ) );
	CHECK( !RMSMath::ControlsRMS( false, true, false ) );// no RMS
	CHECK( !RMSMath::ControlsRMS( false, false, true ) );
	CHECK( !RMSMath::ControlsRMS( false, true, true ) );

	// SAFE brings the arm to rest, and it is maintained with the switch back in AUTO
	bool st = false;
	st = RMSMath::SafingLatch( st, false, false, false );
	CHECK( !st );
	st = RMSMath::SafingLatch( st, true, false, false );// SAFE
	CHECK( st );
	st = RMSMath::SafingLatch( st, false, false, false );// back in AUTO
	CHECK( st );
	st = RMSMath::SafingLatch( st, false, true, false );// CANCEL (momentary)
	CHECK( !st );
	st = RMSMath::SafingLatch( st, false, false, false );// the switch returns to AUTO
	CHECK( !st );
	// CANCEL has no effect while the switch is in SAFE (the condition is still there)
	st = RMSMath::SafingLatch( false, true, true, false );
	CHECK( st );
	// SAFING CAN of the PDRS OVERRIDE display: cancels, and masks the SAFE switch
	st = RMSMath::SafingLatch( true, false, false, true );
	CHECK( !st );
	st = RMSMath::SafingLatch( true, true, false, true );
	CHECK( !st );
	st = RMSMath::SafingLatch( false, true, false, true );
	CHECK( !st );
	// brakes
	CHECK( RMSMath::BrakesApplied( true, true ) );
	CHECK( !RMSMath::BrakesApplied( false, true ) );
	CHECK( RMSMath::BrakesApplied( false, false ) );// no power: the brakes clamp the joints
	CHECK( RMSMath::BrakesApplied( true, false ) );
}

// shutdown, cooldown and restart of the APUs
static void test_apu_model( void )
{
	using namespace apumodel;

	// the limits of the documents
	CHECK_NEAR( UNDERSPEED_PCT, 80.0, 1e-12 );
	CHECK_NEAR( OVERSPEED_PCT, 129.0, 1e-12 );
	CHECK_NEAR( STARTUP_DELAY, 10.5, 1e-12 );
	CHECK_NEAR( LIMIT_INJECTOR_F, 400.0, 1e-12 );
	CHECK_NEAR( LIMIT_PUMP_F, 210.0, 1e-12 );
	CHECK_NEAR( LIMIT_VALVE_F, 200.0, 1e-12 );
	CHECK_NEAR( TargetSpeed( false ), 103.0, 1e-12 );
	CHECK_NEAR( TargetSpeed( true ), 113.0, 1e-12 );

	// underspeed: only after the start delay; overspeed: no delay and strictly above the limit
	CHECK( !UnderspeedTrip( 50.0, 10.4 ) );
	CHECK( UnderspeedTrip( 79.9, 10.5 ) );
	CHECK( !UnderspeedTrip( 80.0, 100.0 ) );
	CHECK( !UnderspeedTrip( 103.0, 100.0 ) );
	CHECK( UnderspeedTrip( 0.0, 1000.0 ) );
	CHECK( OverspeedTrip( 129.1 ) );
	CHECK( !OverspeedTrip( 129.0 ) );
	CHECK( !OverspeedTrip( 113.0 ) );
	// the start ramp (15 %/s) reaches 80 % before the delay ends, so a normal start does not trip
	CHECK( !UnderspeedTrip( 15.0 * STARTUP_DELAY, STARTUP_DELAY ) );

	// lag
	CHECK_NEAR( Relax( 100.0, 500.0, 0.0, 1.0 ), 500.0, 1e-12 );
	CHECK_NEAR( Relax( 100.0, 500.0, 10.0, 0.0 ), 100.0, 1e-9 );
	CHECK_NEAR( Relax( 100.0, 500.0, 10.0, 1e6 ), 500.0, 1e-9 );
	CHECK_NEAR( Relax( 100.0, 500.0, 10.0, 10.0 ), 500.0 - (400.0 * exp( -1.0 )), 1e-9 );

	// a cold APU is safe to start, a running one too
	CHECK( SafeToStart( ColdThermal() ) );
	CHECK( SafeToStart( RunningThermal() ) );
	CHECK_NEAR( DetonationProbability( ColdThermal() ), 0.0, 1e-12 );

	// cooldown after a run: the injector is above the limit right after the shutdown (soakback) and gets to the limit after about 180 minutes
	Thermal t = RunningThermal();
	double tpeak = 0.0;
	bool unsafe10 = false;
	bool safe170 = true;
	bool safe190 = false;
	double inj120 = 0.0;
	for (int s = 1; s <= 200 * 60; s++)
	{
		ThermalStep( t, false, false, 1.0 );
		tpeak = (std::max)( tpeak, t.injector );
		if (s == (10 * 60)) unsafe10 = !SafeToStart( t );
		if (s == (120 * 60)) inj120 = t.injector;
		if (s == (170 * 60)) safe170 = SafeToStart( t );
		if (s == (190 * 60)) safe190 = SafeToStart( t );
		// the fuel pump and the valve module have passive cooling: they are inside the limits during the whole cooldown
		CHECK( t.pump <= LIMIT_PUMP_F );
		CHECK( t.valve <= LIMIT_VALVE_F );
	}
	CHECK( unsafe10 );
	CHECK( inj120 > LIMIT_INJECTOR_F );
	CHECK( !safe170 );
	CHECK( safe190 );
	CHECK( tpeak > 700.0 );
	CHECK( tpeak <= PEAK_INJECTOR_F + 1.0 );

	// water cooling of the injector: a hot injector gets below the limit in 3.5 minutes at most, and heats up again when the water stops
	for (int minutes = 1; minutes <= 150; minutes += 7)
	{
		Thermal h = RunningThermal();
		for (int s = 0; s < minutes * 60; s++) ThermalStep( h, false, false, 1.0 );
		const double before = h.injector;
		double time = 0.0;
		for (int s = 0; (s < 600) && (h.injector > LIMIT_INJECTOR_F); s++)
		{
			ThermalStep( h, false, true, 1.0 );
			time += 1.0;
		}
		if (before > LIMIT_INJECTOR_F)
		{
			CHECK( h.injector <= LIMIT_INJECTOR_F );
			CHECK( time <= COOLING_CYCLE );
		}
		// 3.5 minutes of cooling (the cycle of the checklist) is enough to restart
		for (int s = 0; s < (int)COOLING_CYCLE; s++) ThermalStep( h, false, true, 1.0 );
		CHECK( SafeToStart( h ) );
		// the cooling stops: the injector heats up quickly (more than 1 s before the start is risky)
		const double cooled = h.injector;
		for (int s = 0; s < 120; s++) ThermalStep( h, false, false, 1.0 );
		CHECK( h.injector >= cooled );
		if (minutes < 120) CHECK( h.injector > LIMIT_INJECTOR_F );
	}

	// running keeps the temperatures at the operating values
	Thermal r = ColdThermal();
	for (int s = 0; s < 3600; s++) ThermalStep( r, true, false, 1.0 );
	CHECK_NEAR( r.injector, RUN_INJECTOR_F, 1.0 );
	CHECK_NEAR( r.pump, RUN_PUMP_F, 1.0 );
	CHECK_NEAR( r.valve, RUN_VALVE_F, 1.0 );
	CHECK_NEAR( r.soak, 1.0, 1e-12 );

	// detonation: nothing inside the limits, growing above them, certain at the end of the scale
	Thermal d = ColdThermal();
	d.injector = 400.0;
	CHECK_NEAR( DetonationProbability( d ), 0.0, 1e-12 );
	d.injector = 450.0;
	CHECK_NEAR( DetonationProbability( d ), 1.0 - exp( -50.0 / 150.0 ), 1e-12 );// 28%
	d.injector = 600.0;
	CHECK_NEAR( DetonationProbability( d ), 1.0 - exp( -200.0 / 150.0 ), 1e-12 );// 74%
	d.injector = 5000.0;
	CHECK_NEAR( DetonationProbability( d ), DETONATION_MAX, 1e-12 );
	d = ColdThermal();
	d.pump = 240.0;
	CHECK_NEAR( DetonationProbability( d ), 1.0 - exp( -30.0 / 45.0 ), 1e-12 );
	d = ColdThermal();
	d.valve = 230.0;
	CHECK_NEAR( DetonationProbability( d ), 1.0 - exp( -30.0 / 45.0 ), 1e-12 );

	// leak of a destroyed APU: the scatter of the failure and the variation are limited around the rate of a working APU
	CHECK_NEAR( LeakFactor( 0.0 ), 1.0, 1e-12 );
	CHECK_NEAR( LeakFactor( -100.0 ), 0.5, 1e-12 );
	CHECK_NEAR( LeakFactor( 100.0 ), 1.8, 1e-12 );
	CHECK_NEAR( LeakFactor( 1.0 ), 1.3, 1e-12 );
	CHECK_NEAR( LeakRate( 1.0, 0.0 ), LEAK_RATE, 1e-12 );
	CHECK( (HYD_LOSS_EXPLOSION > HYD_LOSS_FIRE) && (HYD_LOSS_FIRE > 0.0) && (HYD_LOSS_EXPLOSION < 1.0) );

	// fire of a leak: needs air (not on orbit), starts at random, the random fire of the option follows the rate
	CHECK( !AirForFire( 0.0 ) );
	CHECK( !AirForFire( FIRE_MIN_PRESSURE - 1.0 ) );
	CHECK( AirForFire( 101325.0 ) );
	CHECK_NEAR( IgnitionProbability( 1.0, 0.0 ), 0.0, 1e-15 );
	CHECK_NEAR( IgnitionProbability( 1.0, 101325.0 ), 1.0 - std::exp( -IGNITION_RATE ), 1e-12 );
	CHECK_NEAR( IgnitionProbability( 0.0, 101325.0 ), 0.0, 1e-15 );
	CHECK_NEAR( RandomFireProbability( 3600.0, 0.0, 101325.0 ), 0.0, 1e-15 );
	CHECK_NEAR( RandomFireProbability( 3600.0, 1.0, 0.0 ), 0.0, 1e-15 );
	CHECK_NEAR( RandomFireProbability( 3600.0, 1.0, 101325.0 ), 1.0 - std::exp( -1.0 ), 1e-12 );
	CHECK( (EXPLOSION_IGNITES > 0.0) && (EXPLOSION_IGNITES < 1.0) && (BOOM_TIME > 0.0) && (DEBRIS_TIME > BOOM_TIME) );
	CHECK( LeakRate( 1.0, 100.0 ) <= (LEAK_RATE * 1.3 + 1e-12) );
	CHECK( LeakRate( 1.0, -100.0 ) >= (LEAK_RATE * 0.7 - 1e-12) );
	CHECK( (LEAK_RATE * FIRE_SPREAD_TIME) < 350.0 );// the fire check comes while there is still fuel
	CHECK_NEAR( COLLATERAL_EXPLOSION, 0.5, 1e-12 );
	CHECK_NEAR( COLLATERAL_FIRE, 0.15, 1e-12 );
	for (int i = 0; i < 100; i++)
	{
		Thermal a = ColdThermal();
		a.injector = frand( 70.0, 900.0 );
		Thermal b = a;
		b.injector = a.injector + frand( 0.0, 100.0 );
		CHECK( DetonationProbability( b ) >= DetonationProbability( a ) );
		CHECK( (DetonationProbability( a ) > 0.0) == !SafeToStart( a ) );
	}

	// the water of the injector cooling: 21 minutes of flow, six cycles of 3.5 minutes
	CHECK_NEAR( WATER_FLOW_TIME, 21.0 * 60.0, 1e-9 );
	double water = WATER_FLOW_TIME;
	for (int cycle = 0; cycle < 6; cycle++)
	{
		CHECK( water > 0.0 );
		for (int s = 0; s < (int)COOLING_CYCLE; s++) water = WaterAfter( water, 1, 1.0 );
	}
	CHECK_NEAR( water, 0.0, 1e-9 );
	CHECK_NEAR( WaterAfter( 100.0, 2, 10.0 ), 80.0, 1e-12 );// two valves use it twice as fast
	CHECK_NEAR( WaterAfter( 5.0, 3, 10.0 ), 0.0, 1e-12 );
	CHECK_NEAR( WaterAfter( 100.0, 0, 10.0 ), 100.0, 1e-12 );
}

// reaction of the payload that the arm moves on the orbiter
static void test_rms_reaction( void )
{
	using namespace rmsreact;

	// force and torque
	CHECK_VEC( ReactionForce( 1000.0, _V( 0.1, -0.2, 0.0 ) ), _V( -100.0, 200.0, 0.0 ), 1e-9 );
	CHECK_VEC( ReactionForce( 0.0, _V( 1.0, 1.0, 1.0 ) ), _V( 0.0, 0.0, 0.0 ), 1e-12 );
	CHECK_VEC( ReactionTorque( _V( 0.0, 10.0, 0.0 ), _V( 5.0, 0.0, 0.0 ) ), _V( 0.0, 0.0, -50.0 ), 1e-9 );

	// the estimator needs two steps of data
	AccelEstimator e;
	VECTOR3 a;
	CHECK( !e.Update( _V( 0.0, 0.0, 0.0 ), 0.1, a ) );
	CHECK( !e.Update( _V( 0.1, 0.0, 0.0 ), 0.1, a ) );
	CHECK( e.Update( _V( 0.2, 0.0, 0.0 ), 0.1, a ) );
	CHECK( !e.Update( _V( 0.3, 0.0, 0.0 ), 0.0, a ) );// no time step
	// a point that moves with constant velocity has no acceleration
	for (int i = 0; i < 100; i++) e.Update( _V( 0.1 * (3 + i), 0.0, 0.0 ), 0.1, a );
	CHECK_VEC( a, _V( 0.0, 0.0, 0.0 ), 1e-9 );
	// a point at rest has none
	AccelEstimator r;
	for (int i = 0; i < 50; i++)
	{
		const bool got = r.Update( _V( 1.0, 2.0, 3.0 ), 0.05, a );
		if (i >= 2)
		{
			CHECK( got );
			CHECK( length( a ) < 1e-12 );
		}
	}
	// reset
	e.Reset();
	CHECK( !e.Update( _V( 5.0, 5.0, 5.0 ), 0.1, a ) );

	// constant acceleration: the estimate converges to it
	AccelEstimator c;
	const double a0 = 0.03;
	const double dt = 0.05;
	for (int i = 0; i < 400; i++)
	{
		const double t = i * dt;
		c.Update( _V( 0.5 * a0 * t * t, 0.0, 0.0 ), dt, a );
	}
	CHECK_NEAR( a.x, a0, 1e-3 );

	// a jump of the position (the grapple, the release) is limited
	AccelEstimator j;
	for (int i = 0; i < 20; i++) j.Update( _V( 0.0, 0.0, 0.0 ), 0.05, a );
	j.Update( _V( 5.0, 0.0, 0.0 ), 0.05, a );
	CHECK( length( a ) <= MAX_ACCELERATION + 1e-12 );
	// ... and it gives no force at all, neither in that step nor after it (a velocity of 1000 m/s in the filter kept the limit of the acceleration for seconds)
	{
		AccelEstimator s;
		double impulse = 0.0;
		VECTOR3 p = _V( 0.0, 0.0, 0.0 );
		for (int i = 0; i < 2000; i++)
		{
			if (i == 3) p = _V( 11.7, 0.0, 0.0 );// jump of 11.7 m in a step of 0.01 s
			VECTOR3 acc;
			if (s.Update( p, 0.01, acc )) impulse += length( acc ) * 0.01;
			else CHECK( length( acc ) == 0.0 );
		}
		CHECK_NEAR( impulse, 0.0, 1e-9 );
	}

	// conservation of the momentum: the payload (m) moves relative to the orbiter (M) from rest to rest, the orbiter receives the opposite
	// impulses and the center of mass of both does not move
	const double M = 100000.0;
	const double m = 11000.0;
	const double T = 30.0;
	const double travel = 2.0;
	AccelEstimator est;
	double xs = 0.0;// orbiter position
	double vs = 0.0;// orbiter velocity
	double impulse = 0.0;
	double maxv = 0.0;
	double xrel_end = 0.0;
	for (int i = 0; i <= (int)((T + 20.0) / dt); i++)
	{
		const double t = i * dt;
		// payload relative to the orbiter: smooth move (cosine) of 2 m in 30 s
		const double xrel = (t >= T) ? travel : (travel * 0.5 * (1.0 - cos( 3.14159265358979 * t / T )));
		xrel_end = xrel;
		VECTOR3 acc;
		if (est.Update( _V( xrel, 0.0, 0.0 ), dt, acc ))
		{
			const VECTOR3 f = ReactionForce( m, acc );
			impulse += f.x * dt;
			vs += (f.x / M) * dt;
			xs += vs * dt;
			maxv = (std::max)( maxv, fabs( vs ) );
		}
	}
	// the net impulse is zero when the payload is at rest again, and so is the velocity of the orbiter
	CHECK_NEAR( impulse, 0.0, 5.0 );
	CHECK_NEAR( vs, 0.0, 1e-4 );
	CHECK( maxv > 1e-4 );// the orbiter did move
	// the orbiter moved opposite to the payload, in the ratio of the masses (a lag of the filter, a few percent)
	const double expected = -(m / M) * xrel_end;
	CHECK_NEAR( xs, expected, fabs( expected ) * 0.08 );
}

// the configuration files of the controllers of the repository (examples) must be valid: the names of the inputs, the columns and the numbers
static void test_controller_examples( const char* root )
{
	if ((root == NULL) || (root[0] == 0)) return;
	const char* const files[] = {"FPV_Radio.cfg.example", "Gamepad_THC.cfg.example", "Joystick1.cfg.example", "Joystick2.cfg.example", "Pedals.cfg.example", "T16000M_RHC.cfg.example", "T16000M_THC.cfg.example"};
	for (const char* name : files)
	{
		const std::string path = std::string( root ) + "/Config/SSV/Controllers/" + name;
		std::ifstream f( path );
		CHECK( f.is_open() );
		if (!f.is_open()) continue;
		std::string line;
		int connections = 0;
		while (std::getline( f, line ))
		{
			const std::vector<std::string> col = ctrlmath::SplitColumns( line );
			if (col.empty()) continue;
			std::string input = col[0];
			for (auto& c : input) c = (char)toupper( (unsigned char)c );
			if (input == "INPUT") continue;// header
			CHECK( col.size() >= 2 );
			if (col.size() < 2) continue;

			int type = -1;
			int id = -1;
			CHECK( ctrlmath::ParseSubDevice( col[1], type, id ) );
			if (input.compare( 0, 6, "PANEL " ) == 0)
			{
				std::string panel, comp;
				int act = -1;
				CHECK( ctrlmath::ParsePanelInput( col[0], panel, comp, act ) );
			}
			else
			{
				bool known = false;
				for (int i = 0; i < ctrlmath::INPUT_NAME_COUNT; i++) known = known || (input == ctrlmath::INPUT_NAMES[i]);
				CHECK( known );
				if (!known) printf( "unknown input %s in %s\n", input.c_str(), name );
				// REVERSE, DEADZONE, EXPO and CENTER
				if ((col.size() > 2) && !col[2].empty()) CHECK( (ctrlmath::Lower( col[2] ) == "true") || (ctrlmath::Lower( col[2] ) == "false") );
				if ((col.size() > 3) && !col[3].empty()) { const double v = atof( col[3].c_str() ); CHECK( (v >= 0.0) && (v <= 0.9) ); }
				if ((col.size() > 4) && !col[4].empty()) { const double v = atof( col[4].c_str() ); CHECK( (v >= 0.0) && (v <= 1.0) ); }
				if ((col.size() > 5) && !col[5].empty()) { const double v = atof( col[5].c_str() ); CHECK( (v >= -1.0) && (v <= 1.0) ); }
			}
			connections++;
		}
		CHECK( connections > 0 );
	}
}

static void test_controller_parsers( void )
{
	int type = -1;
	int id = -1;
	CHECK( ctrlmath::ParseSubDevice( "Axis X", type, id ) && (type == ctrlmath::SUB_AXIS) && (id == ctrlmath::AX_X) );
	CHECK( ctrlmath::ParseSubDevice( "  axis rz ", type, id ) && (type == ctrlmath::SUB_AXIS) && (id == ctrlmath::AX_RZ) );
	CHECK( ctrlmath::ParseSubDevice( "Slider 0", type, id ) && (id == ctrlmath::AX_SLIDER0) );
	CHECK( ctrlmath::ParseSubDevice( "Slider 1", type, id ) && (id == ctrlmath::AX_SLIDER1) );
	CHECK( ctrlmath::ParseSubDevice( "Button 0", type, id ) && (type == ctrlmath::SUB_BUTTON) && (id == 0) );
	CHECK( ctrlmath::ParseSubDevice( "Button 127", type, id ) && (id == 127) );
	CHECK( !ctrlmath::ParseSubDevice( "Button 128", type, id ) );
	CHECK( !ctrlmath::ParseSubDevice( "Button -1", type, id ) );
	CHECK( !ctrlmath::ParseSubDevice( "Button x", type, id ) );
	CHECK( ctrlmath::ParseSubDevice( "POV 0H", type, id ) && (type == ctrlmath::SUB_POV) && (id == 0) );
	CHECK( ctrlmath::ParseSubDevice( "POV 0V", type, id ) && (id == 1) );
	CHECK( ctrlmath::ParseSubDevice( "POV 3V", type, id ) && (id == 7) );
	CHECK( !ctrlmath::ParseSubDevice( "POV 4H", type, id ) );
	CHECK( !ctrlmath::ParseSubDevice( "POV 0X", type, id ) );
	CHECK( !ctrlmath::ParseSubDevice( "Axis Q", type, id ) );
	CHECK( !ctrlmath::ParseSubDevice( "Slider 2", type, id ) );
	CHECK( !ctrlmath::ParseSubDevice( "", type, id ) );

	std::string panel, comp;
	int act = -1;
	CHECK( ctrlmath::ParsePanelInput( "PANEL A8A1/BRAKES:UP", panel, comp, act ) && (panel == "A8A1") && (comp == "BRAKES") && (act == ctrlmath::ACT_UP) );
	CHECK( ctrlmath::ParsePanelInput( "PANEL A8A1/AUTO SEQ:toggle", panel, comp, act ) && (comp == "AUTO SEQ") && (act == ctrlmath::ACT_TOGGLE) );
	CHECK( ctrlmath::ParsePanelInput( "PANEL A8A1/SAFING:HOLD_DOWN", panel, comp, act ) && (act == ctrlmath::ACT_HOLD_DOWN) );
	CHECK( ctrlmath::ParsePanelInput( "PANEL A8A1/ENTER:PRESS", panel, comp, act ) && (act == ctrlmath::ACT_PRESS) );
	CHECK( !ctrlmath::ParsePanelInput( "PANEL A8A1/ENTER", panel, comp, act ) );
	CHECK( !ctrlmath::ParsePanelInput( "PANEL A8A1:ENTER", panel, comp, act ) );
	CHECK( !ctrlmath::ParsePanelInput( "PANEL /ENTER:PRESS", panel, comp, act ) );
	CHECK( !ctrlmath::ParsePanelInput( "PANEL A8A1/ENTER:PUSH", panel, comp, act ) );
	CHECK( !ctrlmath::ParsePanelInput( "PANEL", panel, comp, act ) );
	// the names of the inputs are unique
	for (int i = 0; i < ctrlmath::INPUT_NAME_COUNT; i++)
	{
		for (int j = i + 1; j < ctrlmath::INPUT_NAME_COUNT; j++) CHECK( std::string( ctrlmath::INPUT_NAMES[i] ) != ctrlmath::INPUT_NAMES[j] );
	}
	// and agree with the enumeration of the inputs
	CHECK( ctrlmath::INPUT_NAME_COUNT == HandControllers::INPUT_COUNT );
	CHECK( std::string( ctrlmath::INPUT_NAMES[HandControllers::RHC_PITCH] ) == "RHC_PITCH" );
	CHECK( std::string( ctrlmath::INPUT_NAMES[HandControllers::RPTA] ) == "RPTA" );
	CHECK( std::string( ctrlmath::INPUT_NAMES[HandControllers::BRAKE_RIGHT] ) == "BRAKE_RIGHT" );
	CHECK( std::string( ctrlmath::INPUT_NAMES[HandControllers::RMS_CONTROL_TOGGLE] ) == "RMS_CONTROL_TOGGLE" );
	// the actions have the values of the enumeration of the code
	CHECK( ctrlmath::ACT_UP == HandControllers::PA_UP );
	CHECK( ctrlmath::ACT_PRESS == HandControllers::PA_PRESS );
}

// center of mass and inertia of the orbiter with the arm and its payload
static void test_mass_props( void )
{
	using namespace massprops;

	// the shares of the arm add up to the whole arm
	CHECK_NEAR( SHARE_SHOULDER + SHARE_UPPER_BOOM + SHARE_ELBOW + SHARE_LOWER_BOOM + SHARE_WRIST, 1.0, 1e-12 );
	// all the joints at one place: the center of mass is that place
	CHECK_VEC( ArmCoM( _V( 1.0, 2.0, 3.0 ), _V( 1.0, 2.0, 3.0 ), _V( 1.0, 2.0, 3.0 ), _V( 1.0, 2.0, 3.0 ) ), _V( 1.0, 2.0, 3.0 ), 1e-12 );
	// the arm along a line: the center is between the shoulder and the end
	{
		const VECTOR3 c = ArmCoM( _V( 0.0, 0.0, 7.0 ), _V( 0.0, 0.0, 0.6 ), _V( 0.0, 0.0, -6.5 ), _V( 0.0, 0.0, -8.3 ) );
		CHECK( (c.z < 7.0) && (c.z > -8.3) );
		const double expected = (7.0 * SHARE_SHOULDER) + (3.8 * SHARE_UPPER_BOOM) + (0.6 * SHARE_ELBOW) + (-2.95 * SHARE_LOWER_BOOM) + (-7.4 * SHARE_WRIST);
		CHECK_NEAR( c.z, expected, 1e-12 );
		CHECK_NEAR( c.x, 0.0, 1e-12 );
	}
	// the center of mass moves rigidly with the joints
	{
		const VECTOR3 sh = _V( 0.3, -0.2, 5.0 ), el = _V( 1.0, 2.0, -1.0 ), wr = _V( -3.0, 0.5, -4.0 ), ee = _V( -3.5, 0.6, -6.0 );
		const VECTOR3 d = _V( 10.0, -20.0, 5.0 );
		CHECK_VEC( ArmCoM( sh + d, el + d, wr + d, ee + d ), ArmCoM( sh, el, wr, ee ) + d, 1e-12 );
	}

	// two masses
	{
		const Point p[2] = {{100.0, _V( 0.0, 0.0, 0.0 )}, {300.0, _V( 4.0, 0.0, 8.0 )}};
		VECTOR3 cog;
		CHECK_NEAR( Combine( p, 2, cog ), 400.0, 1e-12 );
		CHECK_VEC( cog, _V( 3.0, 0.0, 6.0 ), 1e-12 );
		// nothing
		CHECK_NEAR( Combine( p, 0, cog ), 0.0, 1e-12 );
		CHECK_VEC( cog, _V( 0.0, 0.0, 0.0 ), 1e-12 );
		// a pair of equal and opposite masses has no mass, and no center: the origin
		const Point q[2] = {{5.0, _V( 1.0, 1.0, 1.0 )}, {-5.0, _V( 2.0, 2.0, 2.0 )}};
		CHECK_NEAR( Combine( q, 2, cog ), 0.0, 1e-12 );
		CHECK_VEC( cog, _V( 0.0, 0.0, 0.0 ), 1e-12 );
	}

	// moment of inertia of a point mass
	CHECK_VEC( PointInertia( 2.0, _V( 0.0, 3.0, 4.0 ), _V( 0.0, 0.0, 0.0 ) ), _V( 50.0, 32.0, 18.0 ), 1e-12 );
	CHECK_VEC( PointInertia( 2.0, _V( 1.0, 3.0, 4.0 ), _V( 1.0, 0.0, 0.0 ) ), _V( 50.0, 32.0, 18.0 ), 1e-12 );// only the distance counts
	CHECK_VEC( PointInertia( 0.0, _V( 10.0, 30.0, 40.0 ), _V( 0.0, 0.0, 0.0 ) ), _V( 0.0, 0.0, 0.0 ), 1e-12 );

	// no points: nothing changes
	{
		VECTOR3 cog;
		double mass;
		const VECTOR3 pmi = ComposePMI( _V( 120.0, 108.0, 13.5 ), 80000.0, _V( 0.1, -0.2, 0.3 ), NULL, 0, cog, mass );
		CHECK_VEC( pmi, _V( 120.0, 108.0, 13.5 ), 1e-9 );
		CHECK_VEC( cog, _V( 0.1, -0.2, 0.3 ), 1e-12 );
		CHECK_NEAR( mass, 80000.0, 1e-9 );
	}

	// a payload at the center of mass adds mass and no distance: the moments are the same, divided by the larger mass
	{
		VECTOR3 cog;
		double mass;
		const Point p = {10000.0, _V( 1.0, 2.0, 3.0 )};
		const VECTOR3 pmi = ComposePMI( _V( 100.0, 200.0, 300.0 ), 90000.0, _V( 1.0, 2.0, 3.0 ), &p, 1, cog, mass );
		CHECK_NEAR( mass, 100000.0, 1e-9 );
		CHECK_VEC( cog, _V( 1.0, 2.0, 3.0 ), 1e-12 );
		CHECK_VEC( pmi, _V( 90.0, 180.0, 270.0 ), 1e-9 );
	}

	// a payload away from the center: the center moves and the inertia is the sum with the parallel axes
	{
		VECTOR3 cog;
		double mass;
		const Point p = {100.0, _V( 0.0, 0.0, 10.0 )};
		const VECTOR3 pmi = ComposePMI( _V( 10.0, 20.0, 30.0 ), 1000.0, _V( 0.0, 0.0, 0.0 ), &p, 1, cog, mass );
		CHECK_NEAR( mass, 1100.0, 1e-9 );
		const double zc = 1000.0 / 1100.0;
		CHECK_VEC( cog, _V( 0.0, 0.0, zc ), 1e-12 );
		const double d1 = zc;// base from the new center
		const double d2 = 10.0 - zc;// payload
		CHECK_NEAR( pmi.x, ((1000.0 * (10.0 + (d1 * d1))) + (100.0 * d2 * d2)) / 1100.0, 1e-9 );
		CHECK_NEAR( pmi.y, ((1000.0 * (20.0 + (d1 * d1))) + (100.0 * d2 * d2)) / 1100.0, 1e-9 );
		CHECK_NEAR( pmi.z, 30.0 * 1000.0 / 1100.0, 1e-9 );// on the axis: the roll moment does not change
	}

	// the result does not depend on where the origin is
	{
		const Point p[2] = {{100.0, _V( 1.0, -2.0, 10.0 )}, {-30.0, _V( 0.5, 0.2, 3.0 )}};
		const VECTOR3 d = _V( 123.0, -45.0, 67.0 );
		const Point q[2] = {{100.0, p[0].r + d}, {-30.0, p[1].r + d}};
		VECTOR3 cog1, cog2;
		double m1, m2;
		const VECTOR3 a = ComposePMI( _V( 10.0, 20.0, 30.0 ), 1000.0, _V( 0.2, 0.3, -0.4 ), p, 2, cog1, m1 );
		const VECTOR3 b = ComposePMI( _V( 10.0, 20.0, 30.0 ), 1000.0, _V( 0.2, 0.3, -0.4 ) + d, q, 2, cog2, m2 );
		CHECK_VEC( a, b, 1e-9 );
		CHECK_VEC( cog1 + d, cog2, 1e-9 );
		CHECK_NEAR( m1, m2, 1e-12 );
	}

	// moving the arm as a displacement (mass where it is, minus mass where it was) is the same as the vessel made from the hull and the arm in the new place
	{
		const VECTOR3 hullPMI = _V( 110.0, 105.0, 12.0 );
		const double hullMass = 70000.0;
		const VECTOR3 hullCoG = _V( 0.0, 0.5, -1.0 );
		const double armMass = ARM_MASS;
		const VECTOR3 stowed = _V( -2.7, 0.7, 0.0 );
		const VECTOR3 now = _V( -2.0, 3.0, -9.0 );

		// the vessel in the default configuration (the arm stowed)
		VECTOR3 cog0;
		double mass0;
		const Point armStowedPoint = {armMass, stowed};
		const VECTOR3 pmi0 = ComposePMI( hullPMI, hullMass, hullCoG, &armStowedPoint, 1, cog0, mass0 );

		// the vessel built with the arm in the new place
		VECTOR3 cogDirect;
		double massDirect;
		const Point armNowPoint = {armMass, now};
		const VECTOR3 pmiDirect = ComposePMI( hullPMI, hullMass, hullCoG, &armNowPoint, 1, cogDirect, massDirect );

		// the displacement
		const Point disp[2] = {{armMass, now}, {-armMass, stowed}};
		VECTOR3 cogDisp;
		double massDisp;
		const VECTOR3 pmiDisp = ComposePMI( pmi0, mass0, cog0, disp, 2, cogDisp, massDisp );
		CHECK_NEAR( massDisp, massDirect, 1e-9 );
		CHECK_VEC( cogDisp, cogDirect, 1e-9 );
		CHECK_VEC( pmiDisp, pmiDirect, 1e-9 );

		// the arm where it was: nothing changes
		const Point none[2] = {{armMass, stowed}, {-armMass, stowed}};
		VECTOR3 cogNone;
		double massNone;
		const VECTOR3 pmiNone = ComposePMI( pmi0, mass0, cog0, none, 2, cogNone, massNone );
		CHECK_VEC( pmiNone, pmi0, 1e-9 );
		CHECK_VEC( cogNone, cog0, 1e-12 );
		CHECK_NEAR( massNone, mass0, 1e-9 );

		// the center moves in the direction of the displacement and in the right proportion
		CHECK_VEC( cogDisp - cog0, (now - stowed) * (armMass / mass0), 1e-9 );
	}

	// a payload on a long arm: more moment about the axes perpendicular to the arm, none about the arm itself
	{
		VECTOR3 cog;
		double mass;
		const Point hubble = {11000.0, _V( 0.0, 0.0, -15.0 )};
		const VECTOR3 pmi = ComposePMI( _V( 120.0, 109.0, 13.5 ), 78000.0, _V( 0.0, 0.0, 0.0 ), &hubble, 1, cog, mass );
		CHECK( pmi.x > 120.0 * 78000.0 / mass );// pitch
		CHECK( pmi.y > 109.0 * 78000.0 / mass );// yaw
		CHECK_NEAR( pmi.z, 13.5 * 78000.0 / mass, 1e-9 );// roll (the arm is along the roll axis)
		CHECK( cog.z < 0.0 );
	}

	// added inertia, with the mass already in the vessel
	{
		const Point p[2] = {{11000.0, _V( 0.0, 0.0, -15.0 )}, {0.0, _V( 1.0, 1.0, 1.0 )}};
		const VECTOR3 added = AddedPMI( p, 2, _V( 0.0, 0.0, 0.0 ), 100000.0 );
		CHECK_NEAR( added.x, 11000.0 * 225.0 / 100000.0, 1e-9 );
		CHECK_NEAR( added.y, 11000.0 * 225.0 / 100000.0, 1e-9 );
		CHECK_NEAR( added.z, 0.0, 1e-12 );
		CHECK_VEC( AddedPMI( p, 2, _V( 0.0, 0.0, 0.0 ), 0.0 ), _V( 0.0, 0.0, 0.0 ), 1e-12 );
		CHECK_VEC( AddedPMI( NULL, 0, _V( 0.0, 0.0, 0.0 ), 1000.0 ), _V( 0.0, 0.0, 0.0 ), 1e-12 );
		// the arm in the same place with opposite masses adds nothing
		const Point pair[2] = {{ARM_MASS, _V( 1.0, 2.0, 3.0 )}, {-ARM_MASS, _V( 1.0, 2.0, 3.0 )}};
		CHECK_VEC( AddedPMI( pair, 2, _V( 5.0, 5.0, 5.0 ), 80000.0 ), _V( 0.0, 0.0, 0.0 ), 1e-12 );
	}
}

// cabin lighting: levels of the floodlights and of the panels
static void test_lighting( void )
{
	using namespace lightmath;

	// knob and voltage
	CHECK_NEAR( KnobFromVolts( 0.0 ), 0.0, 1e-12 );
	CHECK_NEAR( KnobFromVolts( 2.5 ), 0.5, 1e-12 );
	CHECK_NEAR( KnobFromVolts( 5.0 ), 1.0, 1e-12 );
	CHECK_NEAR( KnobFromVolts( -3.0 ), 0.0, 1e-12 );
	CHECK_NEAR( KnobFromVolts( 9.0 ), 1.0, 1e-12 );

	// the level follows the knob: the OFF/DIM stop is the lowest level, BRT the full level, and it grows with the knob
	CHECK_NEAR( KnobLevel( 0.0 ), 0.0, 1e-12 );
	CHECK_NEAR( KnobLevel( 1.0 ), 1.0, 1e-12 );
	CHECK_NEAR( KnobLevel( 0.5 ), 0.25, 1e-12 );
	CHECK_NEAR( KnobLevel( -1.0 ), 0.0, 1e-12 );
	CHECK_NEAR( KnobLevel( 2.0 ), 1.0, 1e-12 );
	double last = -1.0;
	for (int i = 0; i <= 100; i++)
	{
		const double l = KnobLevel( i / 100.0 );
		CHECK( l >= last );
		CHECK( (l >= 0.0) && (l <= 1.0) );
		last = l;
	}

	// glareshield: BRIGHT, VAR (knob), OFF
	CHECK_NEAR( GlareshieldLevel( true, false, 0.0 ), 1.0, 1e-12 );// BRIGHT ignores the knob
	CHECK_NEAR( GlareshieldLevel( true, true, 0.3 ), 1.0, 1e-12 );// BRIGHT has priority
	CHECK_NEAR( GlareshieldLevel( false, true, 0.5 ), 0.25, 1e-12 );
	CHECK_NEAR( GlareshieldLevel( false, true, 1.0 ), 1.0, 1e-12 );
	CHECK_NEAR( GlareshieldLevel( false, true, 0.0 ), 0.0, 1e-12 );
	CHECK_NEAR( GlareshieldLevel( false, false, 1.0 ), 0.0, 1e-12 );// OFF ignores the knob

	// stations: ON/OFF and a knob
	CHECK_NEAR( SwitchedLevel( true, 1.0 ), 1.0, 1e-12 );
	CHECK_NEAR( SwitchedLevel( true, 0.5 ), 0.25, 1e-12 );
	CHECK_NEAR( SwitchedLevel( false, 1.0 ), 0.0, 1e-12 );

	// seat and console: one of them, never both
	CHECK( SeatConsoleSelect( true, false ) == SC_SEAT );
	CHECK( SeatConsoleSelect( false, true ) == SC_CONSOLE );
	CHECK( SeatConsoleSelect( false, false ) == SC_OFF );
	CHECK( SeatConsoleSelect( true, true ) == SC_OFF );

	// panels: BRT is the panel as it is in the mesh
	CHECK_NEAR( PanelScale( 1.0 ), 1.0, 1e-12 );
	CHECK_NEAR( PanelScale( 0.0 ), 0.0, 1e-12 );
	CHECK( PanelScale( 0.7 ) < PanelScale( 0.8 ) );

	// emitter
	CHECK_NEAR( EmitterIntensity( 0.5, 1.0 ), 0.5, 1e-12 );
	CHECK_NEAR( EmitterIntensity( 0.5, 3.0 ), 1.0, 1e-12 );
	CHECK_NEAR( EmitterIntensity( -0.5, 1.0 ), 0.0, 1e-12 );

	// ramp: the level does not jump and does not pass the target
	CHECK_NEAR( Ramp( 0.0, 1.0, 4.0, 0.1 ), 0.4, 1e-12 );
	CHECK_NEAR( Ramp( 0.9, 1.0, 4.0, 0.1 ), 1.0, 1e-12 );
	CHECK_NEAR( Ramp( 1.0, 0.0, 4.0, 0.1 ), 0.6, 1e-12 );
	CHECK_NEAR( Ramp( 0.1, 0.0, 4.0, 0.1 ), 0.0, 1e-12 );
	CHECK_NEAR( Ramp( 0.5, 0.5, 4.0, 0.1 ), 0.5, 1e-12 );
	{
		double l = 0.0;
		int steps = 0;
		while ((l < 1.0) && (steps < 1000))
		{
			l = Ramp( l, 1.0, 4.0, 0.05 );
			steps++;
		}
		CHECK( steps == 5 );// 0.25 s from dark to full
	}
}

// the table of the panels of the cabin lighting (made from the meshes by tools/gen-cabin-lighting-data.py)
static void test_panel_lighting_table( void )
{
	using namespace cabinlight;

	CHECK( PANEL_LIGHTING_COUNT > 40 );
	int perChannel[PNL_COUNT] = {0, 0, 0, 0, 0, 0, 0};
	for (int i = 0; i < PANEL_LIGHTING_COUNT; i++)
	{
		const PanelLightingEntry& e = PANEL_LIGHTING[i];
		const std::string name = e.mesh;
		CHECK( name.compare( 0, 5, "panel" ) == 0 );// the meshes are panelXX
		bool lower = true;
		for (char c : name) lower = lower && !((c >= 'A') && (c <= 'Z'));
		CHECK( lower );
		CHECK( (e.channel >= PNL_NONE) && (e.channel < PNL_COUNT) );
		CHECK( e.channel != PNL_NONE );// every panel has a channel
		CHECK( e.dimMask != 0 );// and something to dim: the first material of a panel mesh is the panel
		CHECK( (e.dimMask & 1) != 0 );
		perChannel[e.channel]++;
		// the names are unique
		for (int j = i + 1; j < PANEL_LIGHTING_COUNT; j++) CHECK( name != PANEL_LIGHTING[j].mesh );
	}
	// all the channels have panels (the knobs have something to dim)
	for (int c = PNL_LEFT_CTR; c < PNL_COUNT; c++) CHECK( perChannel[c] > 0 );
	// the panels of the controls are in their channels
	for (int i = 0; i < PANEL_LIGHTING_COUNT; i++)
	{
		const std::string name = PANEL_LIGHTING[i].mesh;
		if (name == "panelo6") CHECK( PANEL_LIGHTING[i].channel == PNL_LEFT_OVHD );
		if (name == "panelo8") CHECK( PANEL_LIGHTING[i].channel == PNL_RIGHT_OVHD );
		if (name == "panelr10") CHECK( PANEL_LIGHTING[i].channel == PNL_MS );
		if (name == "panela6u") CHECK( PANEL_LIGHTING[i].channel == PNL_OS );
	}
}

// free movement of the head in the cockpit
static void test_free_move( void )
{
	// the lower deck has no limits
	{
		VECTOR3 o = _V( 0.0, 0.0, 0.0 );
		VECTOR3 u = _V( 0.0, 0.0, 0.0 );
		for (int i = 0; i < 200; i++)
		{
			o = freemove::Move( o, 0.0, 1.0, 0.0, _V( 0.0, 0.0, 1.0 ), 0.1, true );
			u = freemove::Move( u, 0.0, 1.0, 0.0, _V( 0.0, 0.0, 1.0 ), 0.1, false );
		}
		CHECK( o.x > 2.0 );
		CHECK( u.x <= freemove::RADIUS_HORIZONTAL + 1e-9 );
	}
	using namespace freemove;

	// limits: inside is not changed, outside is moved to the surface on the line to the center
	CHECK_VEC( ClampOffset( _V( 0.1, 0.1, 0.1 ) ), _V( 0.1, 0.1, 0.1 ), 1e-12 );
	{
		const VECTOR3 o = ClampOffset( _V( 2.0, 0.0, 0.0 ) );
		CHECK_NEAR( o.x, RADIUS_HORIZONTAL, 1e-12 );
		CHECK_NEAR( o.y, 0.0, 1e-12 );
		CHECK( IsInside( o ) );
	}
	{
		const VECTOR3 o = ClampOffset( _V( 0.0, 5.0, 0.0 ) );
		CHECK_NEAR( o.y, RADIUS_UP, 1e-12 );
		const VECTOR3 d = ClampOffset( _V( 0.0, -5.0, 0.0 ) );
		CHECK_NEAR( d.y, -RADIUS_DOWN, 1e-12 );
	}
	{
		// the direction is kept
		const VECTOR3 o = ClampOffset( _V( 3.0, 0.0, 4.0 ) );
		CHECK_NEAR( o.x / o.z, 0.75, 1e-12 );
		CHECK_NEAR( sqrt( (o.x * o.x) + (o.z * o.z) ), RADIUS_HORIZONTAL, 1e-12 );
	}
	CHECK( IsInside( _V( 0.0, 0.0, 0.0 ) ) );
	CHECK( !IsInside( _V( 0.8, 0.0, 0.0 ) ) );
	CHECK( !IsInside( _V( 0.0, 0.5, 0.0 ) ) );
	CHECK( IsInside( _V( 0.0, -0.39, 0.0 ) ) );

	// random points: the clamp is always inside, and the points inside do not move
	for (int i = 0; i < 2000; i++)
	{
		const VECTOR3 o = _V( (rand() / (double)RAND_MAX - 0.5) * 3.0, (rand() / (double)RAND_MAX - 0.5) * 3.0, (rand() / (double)RAND_MAX - 0.5) * 3.0 );
		const VECTOR3 c = ClampOffset( o );
		CHECK( IsInside( c ) );
		if (IsInside( o )) CHECK_VEC( c, o, 1e-12 );
		CHECK_VEC( ClampOffset( c ), c, 1e-9 );// idempotent
	}

	// the direction of the view on the horizontal plane
	CHECK_VEC( HorizontalForward( _V( 0.0, 0.0, 1.0 ), _V( 0.0, 0.0, 1.0 ) ), _V( 0.0, 0.0, 1.0 ), 1e-12 );
	CHECK_VEC( HorizontalForward( _V( 1.0, 5.0, 1.0 ), _V( 0.0, 0.0, 1.0 ) ), _V( 0.70710678, 0.0, 0.70710678 ), 1e-7 );// the height of the view does not matter
	CHECK_VEC( HorizontalForward( _V( 0.0, 1.0, 0.0 ), _V( 1.0, 0.0, 0.0 ) ), _V( 1.0, 0.0, 0.0 ), 1e-12 );// looking up: the default direction
	CHECK_VEC( HorizontalForward( _V( 0.0, -1.0, 0.0 ), _V( 0.0, 0.0, 0.0 ) ), _V( 0.0, 0.0, 1.0 ), 1e-12 );// nothing: forward of the vessel
	// right of the view
	CHECK_VEC( RightOf( _V( 0.0, 0.0, 1.0 ) ), _V( 1.0, 0.0, 0.0 ), 1e-12 );
	CHECK_VEC( RightOf( _V( 1.0, 0.0, 0.0 ) ), _V( 0.0, 0.0, -1.0 ), 1e-12 );
	CHECK_VEC( RightOf( _V( 0.0, 0.0, -1.0 ) ), _V( -1.0, 0.0, 0.0 ), 1e-12 );

	// movement
	const VECTOR3 f = _V( 0.0, 0.0, 1.0 );
	{
		const VECTOR3 o = Move( _V( 0.0, 0.0, 0.0 ), 1.0, 0.0, 0.0, f, 0.1 );
		CHECK_VEC( o, _V( 0.0, 0.0, SPEED * 0.1 ), 1e-12 );// forward
		CHECK_VEC( Move( _V( 0.0, 0.0, 0.0 ), -1.0, 0.0, 0.0, f, 0.1 ), _V( 0.0, 0.0, -SPEED * 0.1 ), 1e-12 );
		CHECK_VEC( Move( _V( 0.0, 0.0, 0.0 ), 0.0, 1.0, 0.0, f, 0.1 ), _V( SPEED * 0.1, 0.0, 0.0 ), 1e-12 );// right
		CHECK_VEC( Move( _V( 0.0, 0.0, 0.0 ), 0.0, -1.0, 0.0, f, 0.1 ), _V( -SPEED * 0.1, 0.0, 0.0 ), 1e-12 );
		CHECK_VEC( Move( _V( 0.0, 0.0, 0.0 ), 0.0, 0.0, 1.0, f, 0.1 ), _V( 0.0, SPEED * 0.1, 0.0 ), 1e-12 );// up
	}
	// forward follows the view: looking to the right (+x) forward is +x and right is -z
	{
		const VECTOR3 v = _V( 1.0, 0.0, 0.0 );
		CHECK_VEC( Move( _V( 0.0, 0.0, 0.0 ), 1.0, 0.0, 0.0, v, 0.1 ), _V( SPEED * 0.1, 0.0, 0.0 ), 1e-12 );
		CHECK_VEC( Move( _V( 0.0, 0.0, 0.0 ), 0.0, 1.0, 0.0, v, 0.1 ), _V( 0.0, 0.0, -SPEED * 0.1 ), 1e-12 );
	}
	// opposite keys cancel
	CHECK_VEC( Move( _V( 0.1, 0.0, 0.0 ), 0.0, 0.0, 0.0, f, 0.1 ), _V( 0.1, 0.0, 0.0 ), 1e-12 );
	// the diagonal is not faster
	{
		const VECTOR3 o = Move( _V( 0.0, 0.0, 0.0 ), 1.0, 1.0, 1.0, f, 0.1 );
		CHECK_NEAR( sqrt( (o.x * o.x) + (o.y * o.y) + (o.z * o.z) ), SPEED * 0.1, 1e-12 );
	}
	// the step of time is limited, and a step without time does not move
	CHECK_VEC( Move( _V( 0.0, 0.0, 0.0 ), 1.0, 0.0, 0.0, f, 5.0 ), _V( 0.0, 0.0, SPEED * MAX_STEP ), 1e-12 );
	CHECK_VEC( Move( _V( 0.0, 0.0, 0.0 ), 1.0, 0.0, 0.0, f, 0.0 ), _V( 0.0, 0.0, 0.0 ), 1e-12 );
	// the limit stops the movement
	{
		VECTOR3 o = _V( 0.0, 0.0, 0.0 );
		for (int i = 0; i < 1000; i++)
		{
			o = Move( o, 1.0, 0.0, 0.0, f, 0.05 );
			CHECK( IsInside( o ) );
		}
		CHECK_NEAR( o.z, RADIUS_HORIZONTAL, 1e-9 );
		// and the way back is possible
		o = Move( o, -1.0, 0.0, 0.0, f, 0.1 );
		CHECK( o.z < RADIUS_HORIZONTAL );
	}
	// time to cross the range at the speed of the head (a little less than 3 s for half a meter)
	{
		VECTOR3 o = _V( 0.0, 0.0, 0.0 );
		int steps = 0;
		while ((o.z < (RADIUS_HORIZONTAL - 1e-9)) && (steps < 10000))
		{
			o = Move( o, 1.0, 0.0, 0.0, f, 0.01 );
			steps++;
		}
		CHECK( (steps * 0.01) > 1.0 );
		CHECK( (steps * 0.01) < 2.5 );
	}

	// stations that allow the movement
	CHECK( StationAllowsMove( 0, 12, 18, 200 ) );// CDR
	CHECK( StationAllowsMove( 11, 12, 18, 200 ) );// aft workstation
	CHECK( !StationAllowsMove( 12, 12, 18, 200 ) );// cameras
	CHECK( !StationAllowsMove( 17, 12, 18, 200 ) );
	CHECK( !StationAllowsMove( 18, 12, 18, 200 ) );
	CHECK( StationAllowsMove( 20, 12, 18, 200 ) );// panel positions
	CHECK( StationAllowsMove( 100, 12, 18, 200 ) );// middeck
	CHECK( !StationAllowsMove( 200, 12, 18, 200 ) );// airlock
}

// table of the options, operations on the values and the list of the settings MFD
static void test_settings( const char* root )
{
	using namespace settings;

	// the table
	for (int i = 0; i < COUNT; i++)
	{
		const Item& it = ITEMS[i];
		CHECK( (it.key != NULL) && (strlen( it.key ) > 0) );
		CHECK( (it.label != NULL) && (strlen( it.label ) > 0) && (strlen( it.label ) <= 28) );// it has to fit in a line of the MFD
		CHECK( (it.note != NULL) && (strlen( it.note ) > 0) && (strlen( it.note ) < 110) );// three lines of the MFD
		CHECK( (it.kind >= KIND_BOOL) && (it.kind <= KIND_CHOICE) );
		if (it.kind == KIND_CHOICE)
		{
			CHECK( it.choices != NULL );
			CHECK( (it.min == 0.0) && (it.step == 1.0) );
			for (int c = 0; c <= (int)it.max; c++) CHECK( (it.choices[c] != NULL) && (strlen( it.choices[c] ) > 0) && (strlen( it.choices[c] ) <= 20) );
		}
		else CHECK( it.choices == NULL );
		CHECK( it.min <= it.def );
		CHECK( it.def <= it.max );
		CHECK( it.step > 0.0 );
		CHECK( (it.group >= 0) && (it.group < GROUP_COUNT) );
		if (it.kind == KIND_BOOL) CHECK( (it.min == 0.0) && (it.max == 1.0) );
		for (int j = i + 1; j < COUNT; j++) CHECK( _stricmp( it.key, ITEMS[j].key ) != 0 );// unique keys
		CHECK( FindKey( it.key ) == i );
		// the groups are together
		if (i > 0)
		{
			if (ITEMS[i - 1].group != it.group)
			{
				for (int j = 0; j < i; j++) CHECK( ITEMS[j].group != it.group );
			}
		}
	}
	CHECK( FindKey( "rhcrate" ) == ITEM_RHC_RATE );// not case sensitive
	CHECK( FindKey( "Nothing" ) == -1 );
	int total = 0;
	for (int g = 0; g < GROUP_COUNT; g++) total += CountInGroup( g );
	CHECK( total == COUNT );
	CHECK( std::string( GroupName( GROUP_RMS ) ) == "RMS" );
	CHECK( std::string( GroupName( 99 ) ).empty() );

	// the defaults agree with the constants of the code
	CHECK_NEAR( ITEMS[ITEM_RHC_RATE].def, ::RHC_RATE, 1e-12 );
	CHECK_NEAR( ITEMS[ITEM_RPTA_RATE].def, ::RPTA_RATE, 1e-12 );
	CHECK_NEAR( ITEMS[ITEM_BRAKE_RATE].def, ::BRAKE_RATE, 1e-12 );

	// values
	CHECK_NEAR( Clamp( POSITION_LABEL_TIME, 3.4 ), 3.0, 1e-12 );
	CHECK_NEAR( Clamp( POSITION_LABEL_TIME, 3.6 ), 4.0, 1e-12 );
	CHECK_NEAR( Clamp( POSITION_LABEL_TIME, 100.0 ), 60.0, 1e-12 );
	CHECK_NEAR( Clamp( POSITION_LABEL_TIME, -5.0 ), 0.0, 1e-12 );
	CHECK_NEAR( Clamp( ITEM_RHC_RATE, 0.0 ), 0.05, 1e-12 );
	CHECK_NEAR( Clamp( ITEM_RHC_RATE, 9.0 ), 5.0, 1e-12 );
	CHECK_NEAR( Clamp( ITEM_RHC_RATE, 0.77 ), 0.77, 1e-12 );// a float is not rounded
	CHECK_NEAR( Clamp( CABIN_LIGHTING, 0.7 ), 1.0, 1e-12 );
	CHECK_NEAR( Clamp( CABIN_LIGHTING, 0.2 ), 0.0, 1e-12 );

	// steps
	CHECK_NEAR( Step( CABIN_LIGHTING, 1.0, 1 ), 0.0, 1e-12 );// a boolean toggles
	CHECK_NEAR( Step( CABIN_LIGHTING, 0.0, -1 ), 1.0, 1e-12 );
	CHECK_NEAR( Step( POSITION_LABEL_TIME, 3.0, 1 ), 4.0, 1e-12 );
	CHECK_NEAR( Step( POSITION_LABEL_TIME, 0.0, -1 ), 0.0, 1e-12 );// at the limit
	CHECK_NEAR( Step( POSITION_LABEL_TIME, 60.0, 1 ), 60.0, 1e-12 );
	CHECK( Step( ITEM_RHC_RATE, 0.75, 1 ) == 0.8 );// exactly on the grid, without the error of the sum
	CHECK( Step( ITEM_RHC_RATE, 0.8, -1 ) == 0.75 );
	{
		double v = 0.75;
		for (int i = 0; i < 100; i++) v = Step( ITEM_RHC_RATE, v, 1 );
		CHECK_NEAR( v, 5.0, 1e-12 );
		for (int i = 0; i < 200; i++) v = Step( ITEM_RHC_RATE, v, -1 );
		CHECK_NEAR( v, 0.05, 1e-12 );
	}
	CHECK( Step( ITEM_RHC_RATE, 0.77, 1 ) == 0.8 );// a value out of the grid goes to the grid
	CHECK( IsDefault( ITEM_RHC_RATE, 0.75 ) );
	CHECK( !IsDefault( ITEM_RHC_RATE, 0.8 ) );

	// the choice of the key: circular in both directions, with the names
	CHECK( FREE_MOVE_KEY_COUNT == 6 );
	CHECK_NEAR( ITEMS[VC_FREE_MOVE_KEY].max, FREE_MOVE_KEY_COUNT - 1, 1e-12 );
	CHECK_NEAR( Step( VC_FREE_MOVE_KEY, 0.0, 1 ), 1.0, 1e-12 );
	CHECK_NEAR( Step( VC_FREE_MOVE_KEY, FREE_MOVE_KEY_COUNT - 1, 1 ), 0.0, 1e-12 );
	CHECK_NEAR( Step( VC_FREE_MOVE_KEY, 0.0, -1 ), FREE_MOVE_KEY_COUNT - 1, 1e-12 );
	CHECK_NEAR( Clamp( VC_FREE_MOVE_KEY, 9.0 ), FREE_MOVE_KEY_COUNT - 1, 1e-12 );
	// the keys are different and are not the ones of the THC (arrows, Ins and Del without Ctrl are used, but Ctrl+Ins and Ctrl+Del are free)
	for (int c = 0; c < FREE_MOVE_KEY_COUNT; c++)
	{
		CHECK( freemove::ToggleKey( c ) != 0 );
		for (int d = c + 1; d < FREE_MOVE_KEY_COUNT; d++) CHECK( freemove::ToggleKey( c ) != freemove::ToggleKey( d ) );
		CHECK( freemove::ToggleKey( c ) != OAPI_KEY_M );// that one is Orbiter's map
	}
	CHECK( freemove::ToggleKey( 99 ) == 0 );

	// text
	char buf[32];
	FormatValue( CABIN_LIGHTING, 1.0, buf, sizeof( buf ) );
	CHECK( std::string( buf ) == "ON" );
	FormatValue( CABIN_LIGHTING, 0.0, buf, sizeof( buf ) );
	CHECK( std::string( buf ) == "OFF" );
	FormatValue( POSITION_LABEL_TIME, 3.0, buf, sizeof( buf ) );
	CHECK( std::string( buf ) == "3" );
	FormatValue( ITEM_RHC_RATE, 0.75, buf, sizeof( buf ) );
	CHECK( std::string( buf ) == "0.75" );

	// the list of the MFD
	{
		Editor e;
		CHECK( e.Cursor() == 0 );
		CHECK( !e.IsDirty() );
		for (int i = 0; i < COUNT; i++) CHECK_NEAR( e.Value( i ), ITEMS[i].def, 1e-12 );
		int lastMain = 0;
		for (int i = 0; i < COUNT; i++) lastMain = !IsRealismGroup( ITEMS[i].group ) ? i : lastMain;
		e.Move( -1 );// circular, the options of the page REALISM are not in this list
		CHECK( e.Cursor() == lastMain );
		e.Move( 1 );
		CHECK( e.Cursor() == 0 );
		for (int i = 0; i < e.Lines(); i++) e.Move( 1 );
		CHECK( e.Cursor() == 0 );
		CHECK( e.Lines() + [&]() { Editor r; r.SetRealism( true ); return r.Lines(); }() == COUNT );

		// change and reset of an option (RMS_REACTION is on the page REALISM)
		e.SetRealism( true );
		CHECK( ITEMS[e.Cursor()].group >= FIRST_REALISM_GROUP );
		while (e.Cursor() != RMS_REACTION) e.Move( 1 );
		CHECK( e.Change( 1 ) );
		CHECK_NEAR( e.Value( RMS_REACTION ), 1.0, 1e-12 );
		CHECK( e.IsDirty() );
		e.MarkSaved();
		CHECK( !e.IsDirty() );
		CHECK( e.ResetItem() );
		CHECK_NEAR( e.Value( RMS_REACTION ), 0.0, 1e-12 );
		CHECK( e.IsDirty() );
		CHECK( !e.ResetItem() );// it is already the default
		e.SetRealism( false );
		// a float at the limit does not change, and does not make the list dirty
		e.MarkSaved();
		while (e.Cursor() != POSITION_LABEL_TIME) e.Move( 1 );
		for (int i = 0; i < 70; i++) e.Change( 1 );
		CHECK_NEAR( e.Value( POSITION_LABEL_TIME ), 60.0, 1e-12 );
		e.MarkSaved();
		CHECK( !e.Change( 1 ) );
		CHECK( !e.IsDirty() );

		// all the defaults
		CHECK( e.ResetAll() );
		e.SetRealism( true );
		e.ResetAll();
		e.SetRealism( false );
		for (int i = 0; i < COUNT; i++) CHECK_NEAR( e.Value( i ), ITEMS[i].def, 1e-12 );
		CHECK( !e.ResetAll() );

		// values from the options (limited to the range)
		double v[COUNT];
		for (int i = 0; i < COUNT; i++) v[i] = 1000.0;
		e.SetValues( v );
		for (int i = 0; i < COUNT; i++) CHECK_NEAR( e.Value( i ), ITEMS[i].max, 1e-12 );
		CHECK( !e.IsDirty() );

		// windows: the selected line is always shown
		for (int view = 0; view < 2; view++)
		{
			for (int rows = 1; rows <= (COUNT + 3); rows++)
			{
				Editor w;
				w.SetRealism( view == 1 );
				const int n = w.Lines();
				for (int c = 0; c < n; c++)
				{
					const int first = w.FirstRow( rows );
					CHECK( first >= 0 );
					CHECK( (rows >= n) ? (first == 0) : ((first + rows) <= n) );
					// the position of the selected line in the list
					int pos = 0;
					while (w.IndexAt( pos ) != w.Cursor()) pos++;
					CHECK( (pos >= first) && (pos < (first + ((rows < n) ? rows : n))) );
					w.Move( 1 );
				}
			}
		}
	}

	// groups
	{
		Editor e;
		const int g0 = ITEMS[e.Cursor()].group;
		e.MoveGroup( 1 );
		CHECK( ITEMS[e.Cursor()].group != g0 );
		CHECK( (e.Cursor() == 0) || (ITEMS[e.Cursor() - 1].group != ITEMS[e.Cursor()].group) );// the first line of the group
		const int g1 = ITEMS[e.Cursor()].group;
		e.Move( 1 );
		e.MoveGroup( -1 );// to the first of the group
		CHECK( ITEMS[e.Cursor()].group == g1 );
		CHECK( (e.Cursor() == 0) || (ITEMS[e.Cursor() - 1].group != g1) );
		e.MoveGroup( -1 );// to the previous group
		CHECK( ITEMS[e.Cursor()].group == g0 );
		// going forward through all the groups comes back to the first
		Editor c;
		int groups = 1;
		for (int i = 1; i < COUNT; i++) groups += ((ITEMS[i].group != ITEMS[i - 1].group) && !IsRealismGroup( ITEMS[i].group )) ? 1 : 0;
		for (int i = 0; i < groups; i++) c.MoveGroup( 1 );
		CHECK( c.Cursor() == 0 );
		// the same on the page REALISM
		Editor r;
		r.SetRealism( true );
		const int r0 = r.Cursor();
		int rgroups = 0;
		for (int i = 1; i < COUNT; i++) rgroups += ((ITEMS[i].group != ITEMS[i - 1].group) && IsRealismGroup( ITEMS[i].group )) ? 1 : 0;
		CHECK( rgroups == (GROUP_COUNT - FIRST_REALISM_GROUP) );
		for (int i = 0; i < rgroups; i++) r.MoveGroup( 1 );
		CHECK( r.Cursor() == r0 );
		CHECK( FindKey( "HydraulicTolerances" ) == HYDRAULIC_TOLERANCES );
		CHECK( FindKey( "RCSNoise" ) == RCS_NOISE );
		CHECK( IsRealismGroup( ITEMS[HYDRAULIC_NOISE].group ) && IsRealismGroup( ITEMS[TOLERANCE_SCALE].group ) );
		CHECK( !IsRealismGroup( ITEMS[AUTO_ACTION_LANDING_GEAR].group ) );
	}

	// rows of the page of the failures: 3 systems with 6 kinds each
	CHECK( FAIL_ROWS == 18 );
	for (int row = 0; row < FAIL_ROWS; row++)
	{
		CHECK( (FailSystem( row ) >= 1) && (FailSystem( row ) <= FAIL_SYSTEMS) );
		CHECK( (FailKind( row ) >= 0) && (FailKind( row ) < FAIL_KINDS) );
		CHECK( (((FailSystem( row ) - 1) * FAIL_KINDS) + FailKind( row )) == row );
	}
	// pages of the MFD: circular in both directions
	{
		int p = PAGE_SETTINGS;
		for (int i = 0; i < PAGE_COUNT; i++) p = NextPage( p, 1 );
		CHECK( p == PAGE_SETTINGS );
		CHECK( NextPage( PAGE_SETTINGS, -1 ) == (PAGE_COUNT - 1) );
		CHECK( NextPage( PAGE_COUNT - 1, 1 ) == PAGE_SETTINGS );
	}

	// the options of the file of the repository agree with the table: the same keys and the same defaults
	if ((root != NULL) && (root[0] != 0))
	{
		const std::string path = std::string( root ) + "/Config/Vessels/SSV_OV.cfg";
		std::ifstream f( path );
		CHECK( f.is_open() );
		bool found[COUNT];
		for (int i = 0; i < COUNT; i++) found[i] = false;
		std::string line;
		while (f.is_open() && std::getline( f, line ))
		{
			if (line.empty() || (line[0] == ';')) continue;
			const size_t eq = line.find( '=' );
			if (eq == std::string::npos) continue;
			std::string key = line.substr( 0, eq );
			std::string val = line.substr( eq + 1 );
			while (!key.empty() && (key.back() == ' ')) key.pop_back();
			while (!val.empty() && ((val[0] == ' ') || (val[0] == '\t'))) val.erase( 0, 1 );
			while (!val.empty() && ((val.back() == ' ') || (val.back() == '\r'))) val.pop_back();
			if ((key == "ClassName") || (key == "Module")) continue;
			const int i = FindKey( key.c_str() );
			CHECK( i >= 0 );// every option of the file is in the table
			if (i < 0)
			{
				printf( "option %s of SSV_OV.cfg is not in the table\n", key.c_str() );
				continue;
			}
			found[i] = true;
			double v;
			if (ITEMS[i].kind == KIND_BOOL) v = (_stricmp( val.c_str(), "TRUE" ) == 0) ? 1.0 : 0.0;
			else v = atof( val.c_str() );
			CHECK_NEAR( v, ITEMS[i].def, 1e-9 );// the default of the table is the one of the file
			if (fabs( v - ITEMS[i].def ) > 1e-9) printf( "the default of %s differs in SSV_OV.cfg\n", key.c_str() );
		}
		for (int i = 0; i < COUNT; i++)
		{
			CHECK( found[i] );// every option of the table is in the file
			if (!found[i]) printf( "option %s is not in SSV_OV.cfg\n", ITEMS[i].key );
		}
	}
}

// heating and cooling of the hydraulic fluid and of the lube oil, water spray boiler, failures from overheating
static hydmodel::Input hyd_input( double speed, double load, int controller, bool n2 )
{
	hydmodel::Input in;
	in.speed = speed;
	in.pumpNorm = true;
	in.load = load;
	in.ambient = hydmodel::AMBIENT;
	in.powered = true;
	in.controller = controller;
	in.n2Open = n2;
	return in;
}

// advances the state with steps of a second; returns the time [s] until the condition is true (or the limit)
template <class Cond>
static double hyd_run( hydmodel::State& s, const hydmodel::Input& in, double limit, Cond cond )
{
	double t = 0.0;
	while (t < limit)
	{
		hydmodel::Step( s, in, 1.0 );
		t += 1.0;
		if (cond( s )) break;
	}
	return t;
}

static void test_hyd_model( void )
{
	using namespace hydmodel;

	// the numbers of the manual
	CHECK_NEAR( OIL_SETPOINT, 250.0, 1e-12 );
	CHECK_NEAR( HYD_SETPOINT, 208.0, 1e-12 );
	CHECK_NEAR( HYD_THROUGH, 210.0, 1e-12 );
	CHECK_NEAR( HYD_BYPASS, 190.0, 1e-12 );
	CHECK_NEAR( WATER_TANK, 142.0, 1e-12 );
	CHECK_NEAR( OIL_LIMIT, 290.0, 1e-12 );
	CHECK_NEAR( HYD_CAUTION, 230.0, 1e-12 );

	// the initial state
	{
		const State s = InitialState();
		CHECK_NEAR( s.hyd, AMBIENT, 1e-12 );
		CHECK_NEAR( s.oil, AMBIENT, 1e-12 );
		CHECK_NEAR( s.water, WATER_TANK, 1e-12 );
		CHECK_NEAR( s.hydQty, QTY_INITIAL, 1e-12 );
		CHECK( !s.seized );
		CHECK( HydraulicsOk( s ) );
		CHECK( !HydCaution( s ) );
		CHECK( !OilCaution( s ) );
	}

	// the commands of the controller
	CHECK_NEAR( HydSprayCommand( 200.0 ), 0.0, 1e-12 );
	CHECK_NEAR( HydSprayCommand( 208.0 ), 0.0, 1e-12 );
	CHECK_NEAR( HydSprayCommand( 211.0 ), 0.5, 1e-12 );
	CHECK_NEAR( HydSprayCommand( 230.0 ), 1.0, 1e-12 );
	CHECK_NEAR( OilSprayCommand( 240.0 ), 0.0, 1e-12 );
	CHECK_NEAR( OilSprayCommand( 254.0 ), 0.5, 1e-12 );
	CHECK_NEAR( OilSprayCommand( 300.0 ), 1.0, 1e-12 );
	// the bypass valve: hysteresis
	CHECK( !BypassLogic( false, 200.0 ) );
	CHECK( BypassLogic( false, 210.0 ) );
	CHECK( BypassLogic( true, 200.0 ) );// it stays through the boiler between 190 and 210
	CHECK( !BypassLogic( true, 190.0 ) );
	CHECK( !BypassLogic( false, 205.0 ) );
	// the nitrogen
	CHECK_NEAR( N2Effect( 1.0, true ), 1.0, 1e-12 );
	CHECK_NEAR( N2Effect( 0.15, true ), 0.5, 1e-12 );
	CHECK_NEAR( N2Effect( 0.0, true ), 0.0, 1e-12 );
	CHECK_NEAR( N2Effect( 1.0, false ), 0.0, 1e-12 );
	// the controller
	{
		State s = InitialState();
		Input in = hyd_input( 1.0, 0.5, 1, true );
		CHECK( ControllerWorks( s, in ) );
		in.controller = 0;
		CHECK( !ControllerWorks( s, in ) );
		in.controller = 2;
		CHECK( ControllerWorks( s, in ) );
		s.ctrlFail[1] = true;
		CHECK( !ControllerWorks( s, in ) );
		in.controller = 1;
		CHECK( ControllerWorks( s, in ) );
		in.powered = false;
		CHECK( !ControllerWorks( s, in ) );
	}
	// the units of the sensors
	CHECK_NEAR( TemperatureToVolts( 250.0 ), 2.5, 1e-12 );
	CHECK_NEAR( TemperatureToVolts( 900.0 ), 5.0, 1e-12 );
	CHECK_NEAR( TemperatureToVolts( -5.0 ), 0.0, 1e-12 );
	CHECK_NEAR( VoltsToTemperature( TemperatureToVolts( 213.0 ) ), 213.0, 1e-9 );
	// the C&W: the table of its limits has 2.9 V (290 F, 100 F per volt) for the lube oil and 4.2 V for the hydraulic fluid (230 F)
	CHECK_NEAR( TemperatureToVolts( OIL_LIMIT ), 2.9, 1e-12 );
	CHECK_NEAR( HydTemperatureToCwVolts( HYD_CAUTION ), 4.2, 1e-12 );
	CHECK( HydTemperatureToCwVolts( 229.0 ) < 4.2 );
	CHECK( HydTemperatureToCwVolts( 231.0 ) > 4.2 );
	CHECK_NEAR( HydTemperatureToCwVolts( 1000.0 ), 5.0, 1e-12 );
	CHECK_NEAR( HydTemperatureToCwVolts( -5.0 ), 0.0, 1e-12 );
	// load
	CHECK_NEAR( LoadFromFlight( 0.0, false ), 0.25, 1e-12 );
	CHECK_NEAR( LoadFromFlight( 3000.0, false ), 1.0, 1e-12 );
	CHECK_NEAR( LoadFromFlight( 30000.0, false ), 1.0, 1e-12 );
	CHECK_NEAR( LoadFromFlight( 0.0, true ), 0.7, 1e-12 );// the gimbals of the engines

	// a stopped system goes to the ambient temperature
	{
		State s = InitialState();
		s.hyd = 200.0;
		s.oil = 250.0;
		Input in = hyd_input( 0.0, 0.3, 1, true );
		for (int i = 0; i < 20000; i++) Step( s, in, 1.0 );
		CHECK_NEAR( s.hyd, AMBIENT, 1.0 );
		CHECK_NEAR( s.oil, AMBIENT, 1.0 );
		CHECK_NEAR( s.water, WATER_TANK, 1e-9 );// no spray: the temperatures are below the set points
	}

	// nominal run of an APU with the boiler: the temperatures stay at the set points, and the water lasts
	{
		State s = InitialState();
		const Input in = hyd_input( 1.0, 0.8, 1, true );
		const double tOil250 = hyd_run( s, in, 3600.0, []( const State& st ) {return st.oil >= OIL_SETPOINT;} );
		CHECK( (tOil250 > 600.0) && (tOil250 < 1200.0) );// 10 to 20 minutes, as in the ascent: the boiler starts to spray in the first minutes of flight
		for (int i = 0; i < 2400; i++) Step( s, in, 1.0 );// 40 more minutes
		CHECK( (s.oil > 245.0) && (s.oil < 262.0) );
		CHECK( (s.hyd > 195.0) && (s.hyd < 228.0) );
		CHECK( !HydCaution( s ) );
		CHECK( !OilCaution( s ) );
		CHECK( s.water > 0.0 );// there is water for more than an hour
		CHECK( s.water < WATER_TANK );
		CHECK( !s.seized );
		CHECK_NEAR( s.hydDamage, 0.0, 1e-12 );
		CHECK_NEAR( s.hydQty, QTY_INITIAL, 1e-12 );
		// the spray works with the lube oil (the lube oil needs more) and the water used is in the order of the manual
		const double used = WATER_TANK - s.water;
		CHECK( (used > 20.0) && (used < 140.0) );
	}

	// nothing cools: the lube oil gets to the limit in about 3 minutes from the operating temperature, and the bearings seize
	{
		State s = InitialState();
		s.oil = OIL_SETPOINT;
		s.hyd = 150.0;
		const Input in = hyd_input( 1.0, 0.5, 0, true );// no controller
		const double t290 = hyd_run( s, in, 3600.0, []( const State& st ) {return st.oil > OIL_LIMIT;} );
		CHECK( (t290 > 120.0) && (t290 < 240.0) );
		CHECK( OilCaution( s ) );
		CHECK( !s.seized );
		const double tSeize = hyd_run( s, in, 3600.0, []( const State& st ) {return st.seized;} );
		CHECK( (tSeize > 50.0) && (tSeize < 70.0) );// a minute above the limit
		CHECK( ((t290 + tSeize) > 120.0) && ((t290 + tSeize) < 300.0) );// 2 to 5 minutes from the operating temperature, as in the manual
		CHECK_NEAR( s.water, WATER_TANK, 1e-9 );// no spray
	}

	// the lube oil does not stay above the limit for the seizure if it is cooled in time (the controller is changed)
	{
		State s = InitialState();
		s.oil = OIL_SETPOINT;
		s.hyd = 150.0;
		s.ctrlFail[0] = true;// the controller A failed
		Input in = hyd_input( 1.0, 0.5, 1, true );// A selected
		hyd_run( s, in, 3600.0, []( const State& st ) {return st.oil > 270.0;} );
		CHECK( s.oil > 270.0 );
		CHECK( !s.seized );
		in.controller = 2;// the crew selects B
		for (int i = 0; i < 600; i++) Step( s, in, 1.0 );
		CHECK( (s.oil > 245.0) && (s.oil < 262.0) );// back to the set point
		CHECK( !s.seized );
		CHECK( s.oilOver < OIL_SEIZE_TIME );
		CHECK( s.water < WATER_TANK );
	}

	// failures of the boiler
	{
		// ice in the spray valve of the lube oil
		State s = InitialState();
		s.oil = OIL_SETPOINT;
		s.iceOil = true;
		const Input in = hyd_input( 1.0, 0.5, 1, true );
		hyd_run( s, in, 3600.0, []( const State& st ) {return st.oil > OIL_LIMIT;} );
		CHECK( s.oil > OIL_LIMIT );
		CHECK_NEAR( s.oilDuty, 0.0, 1e-12 );
	}
	{
		// the nitrogen supply is closed: no spray
		State s = InitialState();
		s.oil = OIL_SETPOINT;
		const Input in = hyd_input( 1.0, 0.5, 1, false );
		hyd_run( s, in, 3600.0, []( const State& st ) {return st.oil > OIL_LIMIT;} );
		CHECK( s.oil > OIL_LIMIT );
	}
	{
		// leak of the nitrogen: the spray goes down and it stops
		State s = InitialState();
		s.oil = OIL_SETPOINT;
		s.n2Leak = 1.0 / 900.0;
		const Input in = hyd_input( 1.0, 0.5, 1, true );
		for (int i = 0; i < 600; i++) Step( s, in, 1.0 );
		CHECK( (s.n2 > 0.0) && (s.n2 < 1.0) );
		CHECK( !s.seized );
		const double t = hyd_run( s, in, 7200.0, []( const State& st ) {return st.seized;} );
		CHECK( t < 7200.0 );// in the end the lube oil gets too hot
		CHECK_NEAR( s.n2, 0.0, 1e-9 );
	}
	{
		// no water left
		State s = InitialState();
		s.oil = OIL_SETPOINT;
		s.water = 0.5;
		const Input in = hyd_input( 1.0, 0.5, 1, true );
		hyd_run( s, in, 3600.0, []( const State& st ) {return st.oil > OIL_LIMIT;} );
		CHECK( s.oil > OIL_LIMIT );
		CHECK_NEAR( s.water, 0.0, 1e-12 );
	}

	// the hydraulic fluid: bypass valve and spray
	{
		State s = InitialState();
		const Input in = hyd_input( 1.0, 0.8, 1, true );
		bool through = false;
		bool never = true;
		for (int i = 0; i < 4800; i++)
		{
			Step( s, in, 1.0 );
			if (s.hyd < HYD_BYPASS) CHECK( !s.through );// below 190 F the fluid goes around the boiler
			if (s.through && (s.hyd > HYD_BYPASS)) through = true;
			if (s.hyd > 232.0) never = false;
		}
		CHECK( through );// it was sent through the boiler
		CHECK( never );
		CHECK( (s.hyd > 195.0) && (s.hyd < 225.0) );
	}

	// the heat of the pump at low pressure is less
	{
		Input hi = hyd_input( 1.0, 1.0, 0, true );
		Input lo = hi;
		lo.pumpNorm = false;
		CHECK( HydSteadyNoCooling( lo ) < HydSteadyNoCooling( hi ) );
		CHECK( HydSteadyNoCooling( hi ) > HYD_CAUTION );// at a high load, without cooling, the fluid is in caution
		Input idle = hyd_input( 1.0, 0.25, 0, true );
		CHECK( HydSteadyNoCooling( idle ) < HYD_CAUTION );
		Input stopped = hyd_input( 0.0, 1.0, 0, true );
		CHECK_NEAR( HydSteadyNoCooling( stopped ), AMBIENT, 1e-12 );
	}

	// no cooling of the hydraulic fluid at the highest load: caution at 230 F, and later the seals degrade, the system leaks and loses the pressure
	{
		State s = InitialState();
		s.hyd = 200.0;
		const Input in = hyd_input( 1.0, 1.0, 0, true );
		const double tCaution = hyd_run( s, in, 36000.0, []( const State& st ) {return HydCaution( st );} );
		CHECK( tCaution < 36000.0 );
		CHECK_NEAR( s.hydDamage, 0.0, 1e-12 );// there is no damage below 250 F
		CHECK( HydraulicsOk( s ) );
		hyd_run( s, in, 36000.0, []( const State& st ) {return st.hyd > HYD_SEAL_LIMIT + 1.0;} );
		CHECK( s.hyd > HYD_SEAL_LIMIT );
		const double tLoss = hyd_run( s, in, 100000.0, []( const State& st ) {return !HydraulicsOk( st );} );
		CHECK( tLoss < 100000.0 );
		CHECK( s.hydDamage > 0.0 );
		CHECK( s.hydQty < HYD_QTY_MIN_OK + 1e-3 );
		// the APU does not go on with a leak in the hydraulic system if the boiler works: with the cooling nothing of this happens
		State t = InitialState();
		t.hyd = 200.0;
		const Input good = hyd_input( 1.0, 1.0, 1, true );
		for (int i = 0; i < 3000; i++) Step( t, good, 1.0 );
		CHECK_NEAR( t.hydDamage, 0.0, 1e-12 );
		CHECK( HydraulicsOk( t ) );
		CHECK( t.hyd < HYD_CAUTION );
	}

	// leak of the hydraulic system (failure)
	{
		State s = InitialState();
		s.hydLeak = 1.0 / 900.0;
		const Input in = hyd_input( 0.0, 0.0, 0, true );
		const double t = hyd_run( s, in, 3600.0, []( const State& st ) {return !HydraulicsOk( st );} );
		CHECK( std::fabs( t - ((QTY_INITIAL - HYD_QTY_MIN_OK) * 900.0) ) < 5.0 );// what is above the minimum of the reservoir, in 15 minutes for the whole reservoir
	}

	// time acceleration: a long step gives (almost) what the short steps give
	{
		State a = InitialState();
		State b = InitialState();
		const Input in = hyd_input( 1.0, 0.8, 1, true );
		for (int i = 0; i < 300; i++) Step( a, in, 1.0 );
		for (int i = 0; i < 60; i++) Step( b, in, 5.0 );
		CHECK_NEAR( a.oil, b.oil, 2.0 );
		CHECK_NEAR( a.hyd, b.hyd, 2.0 );
	}
	// no step without time
	{
		State s = InitialState();
		s.oil = 123.0;
		Step( s, hyd_input( 1.0, 1.0, 1, true ), 0.0 );
		CHECK_NEAR( s.oil, 123.0, 1e-12 );
		Step( s, hyd_input( 1.0, 1.0, 1, true ), -1.0 );
		CHECK_NEAR( s.oil, 123.0, 1e-12 );
	}
}

// kinematics of the arm
static double kin_frand( double a, double b )
{
	return a + ((b - a) * (rand() / (double)RAND_MAX));
}

static void test_rms_kin( void )
{
	using namespace rmskin;

	// the arm stowed (all the joints at zero): the same pose that the RMS has at the start
	{
		const double q[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		const Pose p = Forward( q );
		CHECK_VEC( p.posEE, EE_POS, 1e-9 );
		CHECK_VEC( p.dirEE, _V( 0.0, 0.0, -1.0 ), 1e-9 );
		CHECK_VEC( p.rotEE, EE_ROT, 1e-9 );
		const IKPose ik = ForwardIK( q );
		CHECK_VEC( ik.pos, _V( SP_JOINT.z - EE_POS.z, 0.0, 0.0 ), 1e-9 );
		CHECK_VEC( ik.dir, _V( 1.0, 0.0, 0.0 ), 1e-9 );
		CHECK_VEC( ik.rot, _V( 0.0, 0.0, 1.0 ), 1e-9 );
		// (the stowed pose is a pose of the cradle: the inverse kinematics of the booms in a plane does not give the zero angles, as it was)
	}

	// the dimensions of the arm
	CHECK_NEAR( SP_EP_DIST, 6.7, 0.5 );// the upper boom is about 6.4 m
	CHECK_NEAR( EP_WP_DIST, 7.1, 0.5 );// and the lower boom about 7 m
	// a straight arm reaches 15 m from the shoulder
	{
		const double zero[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		CHECK_NEAR( length( ForwardIK( zero ).pos ), 15.316, 0.01 );
	}

	// the rotation of a joint moves the tip in a circle about its axis; the shoulder yaw keeps the distance to the shoulder
	{
		double q[6] = {0.0, 20.0, -30.0, 10.0, 5.0, 0.0};
		const double d0 = length( Forward( q ).posEE - SP_JOINT );
		q[SHOULDER_YAW] = 45.0;
		CHECK_NEAR( length( Forward( q ).posEE - SP_JOINT ), d0, 1e-3 );
		// the wrist roll does not move the tip, and rotates the roll vector about the direction
		const Pose a = Forward( q );
		q[WRIST_ROLL] = 90.0;
		const Pose b = Forward( q );
		CHECK_VEC( a.posEE, b.posEE, 1e-9 );
		CHECK_VEC( a.dirEE, b.dirEE, 1e-9 );
		CHECK_NEAR( dotp( a.rotEE, b.rotEE ), 0.0, 1e-6 );// perpendicular
	}

	// forward and inverse: a pose from random joints is reached again by the inverse kinematics (the pose, not the angles, as the same pose can
	// have different angles)
	int ok = 0;
	for (int i = 0; i < 400; i++)
	{
		const double q[6] = {kin_frand( -80.0, 80.0 ), kin_frand( 10.0, 100.0 ), kin_frand( -140.0, -20.0 ), kin_frand( -70.0, 70.0 ), kin_frand( -60.0, 60.0 ), kin_frand( -150.0, 150.0 )};
		const IKPose ik = ForwardIK( q );
		double a[6];
		if (!Inverse( ik.pos, ik.dir, ik.rot, q, a )) continue;
		ok++;
		const IKPose back = ForwardIK( a );
		CHECK_VEC( back.pos, ik.pos, 2e-3 );
		CHECK_VEC( back.dir, ik.dir, 2e-3 );
		CHECK_VEC( back.rot, ik.rot, 2e-3 );
	}
	CHECK( ok > 150 );// most of the poses of random joints are reachable by the inverse kinematics

	// the inverse kinematics of a point that is too far fails
	{
		const double q[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		double a[6];
		CHECK( !Inverse( _V( 30.0, 0.0, 0.0 ), _V( 1.0, 0.0, 0.0 ), _V( 0.0, 0.0, 1.0 ), q, a ) );
	}
}

static pdrs::Input pdrs_input( const double q[6], int mode )
{
	pdrs::Input in;
	memset( &in, 0, sizeof( in ) );
	in.dt = 0.08;
	in.selected = true;
	in.powerPrimary = true;
	in.powered = true;
	in.modeSwitch = mode;
	in.jointSwitch = -1;
	in.armFree = true;
	in.rateCoarse = true;
	in.massEffect = false;
	in.porOffset = _V( 0.0, 0.0, 0.0 );
	for (int i = 0; i < 3; i++) in.payloadAxes[i] = _V( 0.0, 0.0, 0.0 );
	for (int i = 0; i < 6; i++) in.joint[i] = q[i];
	return in;
}

static void test_pdrs_logic( void )
{
	// single joint drive: the commanded rate is the coarse rate of the joint, then the soft stop is reached
	{
		pdrs::Logic logic;
		double q[6] = {0.0, 10.0, -20.0, 0.0, 0.0, 0.0};
		pdrs::Output out;
		pdrs::Input in = pdrs_input( q, pdrs::MODE_SINGLE );
		in.jointSwitch = rmskin::SHOULDER_PITCH;
		in.directPlus = true;
		logic.Step( in, out );
		CHECK( logic.ModeIs( pdrs::MODE_SINGLE ) );
		CHECK_NEAR( out.rate[rmskin::SHOULDER_PITCH], pdrs::JOINT_COARSE_SPEED[rmskin::SHOULDER_PITCH], 1e-6 );
		CHECK_NEAR( out.rate[rmskin::ELBOW_PITCH], 0.0, 1e-9 );
		CHECK( !out.softStop );
	}

	// the mode of the switch is idle until ENTER is pushed
	{
		pdrs::Logic logic;
		double q[6] = {0.0, 10.0, -20.0, 0.0, 0.0, 0.0};
		pdrs::Output out;
		pdrs::Input in = pdrs_input( q, pdrs::MODE_SINGLE );
		logic.Step( in, out );
		in.modeSwitch = pdrs::MODE_END_EFF;
		logic.Step( in, out );
		CHECK( !logic.ModeIs( pdrs::MODE_END_EFF ) );
		in.enter = true;
		logic.Step( in, out );
		CHECK( logic.ModeIs( pdrs::MODE_END_EFF ) );
	}

	// no power: no commands
	{
		pdrs::Logic logic;
		double q[6] = {0.0, 10.0, -20.0, 0.0, 0.0, 0.0};
		pdrs::Output out;
		pdrs::Input in = pdrs_input( q, pdrs::MODE_SINGLE );
		in.powered = false;
		in.jointSwitch = rmskin::SHOULDER_PITCH;
		in.directPlus = true;
		logic.Step( in, out );
		for (int i = 0; i < 6; i++) CHECK_NEAR( out.rate[i], 0.0, 1e-12 );
	}

	// closed loop: the software commands the rates, the arm integrates them
	{
		pdrs::Logic logic;
		double q[6] = {10.0, 30.0, -80.0, 20.0, 10.0, 0.0};
		pdrs::Output out;
		pdrs::Input in = pdrs_input( q, pdrs::MODE_ORB_UNL );
		logic.Step( in, out );// the mode is taken at the first cycle
		CHECK( logic.ModeIs( pdrs::MODE_ORB_UNL ) );
		VECTOR3 p0, p1;
		MATRIX3 a0;
		logic.GetEEState( p0, a0 );
		in.thc[0] = 5.0;// full deflection, coarse rate: 2 ft/s
		for (int i = 0; i < 50; i++)
		{
			for (int j = 0; j < 6; j++) in.joint[j] = q[j];
			logic.Step( in, out );
			for (int j = 0; j < 6; j++) q[j] += out.rate[j] * in.dt;
		}
		for (int j = 0; j < 6; j++) in.joint[j] = q[j];
		logic.Step( in, out );
		logic.GetEEState( p1, a0 );
		CHECK_NEAR( length( p1 - p0 ), 4.0 * 24.0, 10.0 );// 4 s at 24 in/s (a little less: the software slows the joints that would go too fast)

		// RATE HOLD keeps the rate with the hand controller in detent
		in.rateHoldPB = true;
		logic.Step( in, out );
		in.rateHoldPB = false;
		in.thc[0] = 0.0;
		CHECK( logic.IsRateHold() );
		logic.Step( in, out );
		CHECK( logic.IsRateHold() );
		CHECK( (fabs( out.rate[rmskin::ELBOW_PITCH] ) + fabs( out.rate[rmskin::SHOULDER_PITCH] )) > 0.0 );
	}

	// toward the edge of the workspace: no joint jumps or leaves its limits, the pose does not bend
	{
		const int dirs[3] = {0, 1, 2};
		for (int d = 0; d < 3; d++)
		{
			for (int sign = -1; sign <= 1; sign += 2)
			{
				pdrs::Logic logic;
				double q[6] = {10.0, 30.0, -80.0, 20.0, 10.0, 0.0};
				pdrs::Output out;
				pdrs::Input in = pdrs_input( q, pdrs::MODE_ORB_UNL );
				logic.Step( in, out );
				in.thc[dirs[d]] = 5.0 * sign;
				for (int i = 0; i < 600; i++)
				{
					for (int j = 0; j < 6; j++) in.joint[j] = q[j];
					logic.Step( in, out );
					for (int j = 0; j < 6; j++)
					{
						const double step = out.rate[j] * in.dt;
						CHECK( fabs( step ) <= (pdrs::JOINT_SOFTWARE_SPEED * in.dt) + 1e-3 );
						q[j] += step;
						CHECK( q[j] >= rmskin::JOINT_LIMITS[0][j] - 1e-6 );
						CHECK( q[j] <= rmskin::JOINT_LIMITS[1][j] + 1e-6 );
					}
				}
			}
		}
	}

	// OPR CMD: the arm goes to the entered position
	{
		pdrs::Logic logic;
		double q[6] = {10.0, 30.0, -80.0, 20.0, 10.0, 0.0};
		pdrs::Output out;
		pdrs::Input in = pdrs_input( q, pdrs::MODE_OPR_CMD );
		logic.Step( in, out );
		pdrs::Status st;
		logic.GetStatus( st );
		char buf[32];
		for (int i = 0; i < 3; i++)
		{
			sprintf_s( buf, "%.3f", st.pos[i] + ((i == 0) ? 10.0 : 0.0) );
			CHECK( logic.ItemInput( 18 + i, buf ) );
			sprintf_s( buf, "%.3f", st.att[i] );
			CHECK( logic.ItemInput( 21 + i, buf ) );
		}
		CHECK( logic.ItemInput( 25, "" ) );
		CHECK( logic.GetControl().cmdck == 1 );
		in.autoProceed = true;
		for (int i = 0; i < 2000; i++)
		{
			for (int j = 0; j < 6; j++) in.joint[j] = q[j];
			logic.Step( in, out );
			for (int j = 0; j < 6; j++) q[j] += out.rate[j] * in.dt;
			if (logic.GetAutoState() == pdrs::AUTO_IDLE && i > 2) break;
		}
		for (int j = 0; j < 6; j++) in.joint[j] = q[j];
		logic.Step( in, out );
		logic.GetStatus( st );
		for (int i = 0; i < 3; i++) CHECK_NEAR( st.pos[i], logic.GetControl().endpos[i], 0.5 );
		CHECK( logic.GetAutoState() == pdrs::AUTO_IDLE );
	}

	// I/O OFF: the software does not command the arm
	{
		pdrs::Logic logic;
		double q[6] = {10.0, 30.0, -80.0, 20.0, 10.0, 0.0};
		pdrs::Output out;
		pdrs::Input in = pdrs_input( q, pdrs::MODE_ORB_UNL );
		in.thc[0] = 5.0;
		CHECK( logic.ItemInput( 6, "" ) );
		logic.Step( in, out );
		for (int i = 0; i < 6; i++) CHECK_NEAR( out.rate[i], 0.0, 1e-12 );
		CHECK( !logic.ModeIs( pdrs::MODE_ORB_UNL ) );
		CHECK( logic.ItemInput( 5, "" ) );
		logic.Step( in, out );
		logic.Step( in, out );
		CHECK( logic.ModeIs( pdrs::MODE_ORB_UNL ) );
		double sum = 0.0;
		for (int i = 0; i < 6; i++) sum += fabs( out.rate[i] );
		CHECK( sum > 0.0 );
	}

	// the frames of the bus
	{
		const double q[6] = {-123.4567, 12.3456, -99.9999, 150.0, -170.5, 300.25};
		pdrs::Input a = pdrs_input( q, pdrs::MODE_PL );
		a.jointSwitch = 4;
		a.enter = a.directMinus = a.rateHoldPB = a.safeSwitch = true;
		a.eeCapture = a.payload = a.massEffect = true;
		a.rhc[0] = 5.0; a.rhc[1] = -2.5; a.thc[2] = 0.123;
		a.payloadMass = 14500.55;
		a.porOffset = _V( 1.25, -0.5, 3.0 );
		a.payloadAxes[0] = _V( 1.0, 0.0, 0.0 );
		a.payloadAxes[1] = _V( 0.0, 0.6, 0.8 );
		a.payloadAxes[2] = _V( 0.0, -0.8, 0.6 );
		unsigned short frame[pdrs::FRAME_IN_WORDS];
		pdrs::PackInput( a, frame );
		pdrs::Input b;
		pdrs::UnpackInput( frame, b );
		CHECK( b.modeSwitch == pdrs::MODE_PL && b.jointSwitch == 4 );
		CHECK( b.enter && b.directMinus && !b.directPlus && b.rateHoldPB && b.safeSwitch && b.powered && b.armFree && b.selected );
		CHECK( b.eeCapture && !b.eeRigid && b.payload && b.massEffect );
		for (int i = 0; i < 6; i++) CHECK_NEAR( b.joint[i], q[i], 1e-4 );
		for (int i = 0; i < 3; i++) CHECK_NEAR( b.rhc[i], a.rhc[i], 2e-4 );
		CHECK_NEAR( b.thc[2], 0.123, 2e-4 );
		CHECK_NEAR( b.payloadMass, 14500.55, 0.01 );
		CHECK_VEC( b.porOffset, a.porOffset, 1e-5 );
		for (int i = 0; i < 3; i++) CHECK_VEC( b.payloadAxes[i], a.payloadAxes[i], 1e-6 );

		pdrs::Output o, p;
		for (int i = 0; i < 6; i++) o.rate[i] = (i - 2.5) * 1.2345;
		o.safingCancel = false; o.ready = true; o.inProg = false; o.softStop = true; o.rateMin = true;
		o.effMode = pdrs::MODE_AUTO_3;
		unsigned short of[pdrs::FRAME_OUT_WORDS];
		pdrs::PackOutput( o, of );
		pdrs::UnpackOutput( of, p );
		for (int i = 0; i < 6; i++) CHECK_NEAR( p.rate[i], o.rate[i], 6e-4 );
		CHECK( !p.safingCancel && p.ready && !p.inProg && p.softStop && p.rateMin && p.effMode == pdrs::MODE_AUTO_3 );
		o.effMode = -1;
		pdrs::PackOutput( o, of );
		pdrs::UnpackOutput( of, p );
		CHECK( p.effMode == -1 );

		// the MCIU: limits of the joints and of the rates, and no motion when the software does not answer
		double j[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		const double r[6] = {1000.0, -1.0, 0.0, 0.0, 0.0, 0.0};
		pdrs::ApplyJointRates( r, false, 1.0, j );
		CHECK_NEAR( j[0], 0.0, 1e-12 );
		pdrs::ApplyJointRates( r, true, 1.0, j );
		CHECK_NEAR( j[0], pdrs::JOINT_MAX_SPEED[0], 1e-9 );
		CHECK_NEAR( j[1], -1.0, 1e-9 );
		for (int i = 0; i < 20; i++) pdrs::ApplyJointRates( r, true, 1.0, j );
		CHECK( j[0] <= rmskin::JOINT_LIMITS[1][0] );
	}

	// parser of the auto sequences
	{
		std::istringstream f( "# test\nSEQ 2\nPT 1 2 3 4 5 6 PAUSE\nPT 7 8 9 10 11 12\nSEQ 99\nPT 1 1 1 1 1 1\n" );
		std::vector<pdrs::AutoPoint> seq[20];
		CHECK( pdrs::ParseAutoSequences( f, seq ) == 1 );
		CHECK( seq[1].size() == 2 );
		CHECK( seq[1][0].pause && !seq[1][1].pause );
		CHECK_NEAR( seq[1][1].pos[2], 9.0, 1e-12 );
	}
}

static void test_hyd_environment( void )
{
	using namespace hydmodel;

	// shadow of the planet
	const double R = 6.371e6;
	CHECK( InPlanetShadow( 0.0, 0.0, -7.0e6, 0.0, 0.0, 1.0, R ) );
	CHECK( !InPlanetShadow( 0.0, 0.0, 7.0e6, 0.0, 0.0, 1.0, R ) );// sunlit side
	CHECK( !InPlanetShadow( 7.0e6, 0.0, -7.0e6, 0.0, 0.0, 1.0, R ) );// behind the planet but outside of the cylinder
	CHECK_NEAR( EnvironmentTemperature( false ), AMBIENT, 1e-12 );
	CHECK_NEAR( EnvironmentTemperature( true ), SHADE_ENV, 1e-12 );

	// the circulation pump (Crew Operations Manual): ON runs it; GPC starts it below 0 F and stops it above 20 F; off with the APU running
	{
		State s = InitialState();
		Input in = hyd_input( 0.0, 0.25, 1, true );
		CHECK( !CircPumpRuns( s, in ) );// the switch is off
		in.circOn = true;
		CHECK( CircPumpRuns( s, in ) );
		in.speed = 0.5;
		CHECK( !CircPumpRuns( s, in ) );// the APU runs: the pump is disconnected
		in.speed = 0.0;
		in.circOn = false;
		in.circGpc = true;
		s.hyd = 30.0;
		CHECK( !CircPumpRuns( s, in ) );// warm: does not start
		s.hyd = -5.0;
		CHECK( CircPumpRuns( s, in ) );// cold: starts
		in.circAllowed = false;
		CHECK( !CircPumpRuns( s, in ) );// another pump runs in GPC
		in.circAllowed = true;
		s.circ = true;
		s.hyd = 10.0;
		CHECK( CircPumpRuns( s, in ) );// between the limits: stays
		s.hyd = 25.0;
		CHECK( !CircPumpRuns( s, in ) );// warm: stops
		s.hyd = 10.0;
		s.circTime = in.circMaxRun + 1.0;
		CHECK( !CircPumpRuns( s, in ) );// the longest run
	}
	CHECK_NEAR( CIRC_MAX_RUN_SYS1, 900.0, 1e-12 );
	CHECK_NEAR( CIRC_MAX_RUN, 600.0, 1e-12 );

	// a stopped system in the shadow gets cold; with the pump in GPC it is kept around 0 to 20 F by the heat exchanger
	{
		State off = InitialState();
		State gpc = InitialState();
		Input in = hyd_input( 0.0, 0.25, 0, true );
		in.env = SHADE_ENV;
		Input ing = in;
		ing.circGpc = true;
		double lowest = 1000.0;
		int starts = 0;
		bool was = false;
		for (int t = 0; t < 20000; t++)
		{
			Step( off, in, 1.0 );
			Step( gpc, ing, 1.0 );
			if (gpc.hyd < lowest) lowest = gpc.hyd;
			if (gpc.circ && !was) starts++;
			was = gpc.circ;
		}
		CHECK( off.hyd < -15.0 );
		CHECK( !off.circ );
		CHECK( starts >= 1 );
		CHECK( lowest > -8.0 );// the pump starts at 0 F and warms the fluid
		CHECK( gpc.hyd > off.hyd );
	}

	// the timer stops the pump before the fluid reaches 20 F, and it waits for the fluid to recover before it starts again
	{
		State s = InitialState();
		s.hyd = -30.0;
		Input in = hyd_input( 0.0, 0.25, 0, true );
		in.env = -100.0;
		in.circGpc = true;
		in.circMaxRun = 100.0;
		int run = 0;
		bool timedOut = false;
		for (int t = 0; t < 1500; t++)
		{
			Step( s, in, 1.0 );
			if (s.circ) run++;
			else if (run > 0) timedOut = true;
		}
		CHECK( timedOut );
		CHECK( (run >= 99) && (run <= 102) );
		CHECK( s.circHold && !s.circ );
	}

	// the vent and the water lines freeze with no heater in the cold, and thaw with the heater on
	{
		State s = InitialState();
		Input in = hyd_input( 0.0, 0.25, 1, true );
		in.env = SHADE_ENV;
		in.heater = false;
		for (int t = 0; t < (int)(VENT_FREEZE_TIME - 10.0); t++) Step( s, in, 1.0 );
		CHECK( !s.ventIce );
		for (int t = 0; t < 20; t++) Step( s, in, 1.0 );
		CHECK( s.ventIce );
		// no spray, whatever the temperature
		s.hyd = 225.0;
		s.through = true;
		Step( s, in, 1.0 );
		CHECK_NEAR( s.hydDuty, 0.0, 1e-12 );
		in.heater = true;
		for (int t = 0; t < (int)(VENT_FREEZE_TIME / VENT_THAW_RATE) + 5; t++) Step( s, in, 1.0 );
		CHECK( !s.ventIce );
	}

	// the sunlit system does not freeze
	{
		State s = InitialState();
		Input in = hyd_input( 0.0, 0.25, 0, true );
		for (int t = 0; t < 7200; t++) Step( s, in, 1.0 );
		CHECK( !s.ventIce );
	}

	// the nitrogen is used as the water goes out
	{
		State s = InitialState();
		s.hyd = 225.0;
		s.through = true;
		Input in = hyd_input( 0.5, 0.5, 1, true );
		const double n2before = s.n2;
		const double waterBefore = s.water;
		for (int t = 0; t < 60; t++) Step( s, in, 1.0 );
		CHECK( s.water < waterBefore );
		CHECK_NEAR( n2before - s.n2, (waterBefore - s.water) * N2_PER_LB, 1e-9 );
	}

	// random failures: the dice are thrown at random intervals
	CHECK_NEAR( NextRollInterval( 0.0 ), ROLL_MIN, 1e-12 );
	CHECK_NEAR( NextRollInterval( 1.0 ), ROLL_MAX, 1e-12 );
	CHECK_NEAR( NextRollInterval( 0.5 ), 0.5 * (ROLL_MIN + ROLL_MAX), 1e-12 );
	CHECK_NEAR( NextRollInterval( 2.0 ), ROLL_MAX, 1e-12 );
	CHECK_NEAR( RollChance( 0.0 ), 0.0, 1e-12 );
	// the rate [1/hour] is kept: throws per hour (3600 s / mean interval) times the chance
	CHECK_NEAR( RollChance( 1.0 ) * 3600.0 / (0.5 * (ROLL_MIN + ROLL_MAX)), 1.0, 1e-9 );
	CHECK( RollChance( 0.05 ) < RollChance( 0.2 ) );
	CHECK( RollChance( 1000.0 ) <= 1.0 );
}

static void test_adi( void )
{
	using namespace adimath;

	// active rotations of the body about its axes (the body axes in the frame are the columns); the matrix M50 to body is the transpose
	const double pi = 3.14159265358979323846;
	auto rx = []( double a ) { Mat3 r = {{{1, 0, 0}, {0, std::cos( a ), -std::sin( a )}, {0, std::sin( a ), std::cos( a )}}}; return r; };
	auto ry = []( double a ) { Mat3 r = {{{std::cos( a ), 0, std::sin( a )}, {0, 1, 0}, {-std::sin( a ), 0, std::cos( a )}}}; return r; };
	auto rz = []( double a ) { Mat3 r = {{{std::cos( a ), -std::sin( a ), 0}, {std::sin( a ), std::cos( a ), 0}, {0, 0, 1}}}; return r; };
	double r, p, y;

	// the quaternion of the attitude processor is the one of the matrix M50 to body: the identity is 0, 0, 0
	Angles( FromQuaternion( 1.0, 0.0, 0.0, 0.0 ), r, p, y );
	CHECK_NEAR( r, 0.0, 1e-9 );
	CHECK_NEAR( p, 0.0, 1e-9 );
	CHECK_NEAR( y, 0.0, 1e-9 );

	// roll to the right, nose up, nose to the right: positive angles
	Angles( Transpose( rx( 30.0 * pi / 180.0 ) ), r, p, y );
	CHECK_NEAR( r, 30.0, 1e-9 );
	CHECK_NEAR( p, 0.0, 1e-9 );
	CHECK_NEAR( y, 0.0, 1e-9 );
	Angles( Transpose( ry( 40.0 * pi / 180.0 ) ), r, p, y );
	CHECK_NEAR( p, 40.0, 1e-9 );
	CHECK_NEAR( r, 0.0, 1e-9 );
	Angles( Transpose( rz( 20.0 * pi / 180.0 ) ), r, p, y );
	CHECK_NEAR( y, 20.0, 1e-9 );
	// negative angles are from 270 to 360
	Angles( Transpose( rx( -10.0 * pi / 180.0 ) ), r, p, y );
	CHECK_NEAR( r, 350.0, 1e-9 );
	// a quaternion of a roll of 30 degrees (the passive rotation is the one of -30)
	{
		const double a = -30.0 * pi / 180.0 / 2.0;
		Angles( FromQuaternion( std::cos( a ), std::sin( a ), 0.0, 0.0 ), r, p, y );
		CHECK_NEAR( r, 30.0, 1e-9 );
	}

	// LVLH: orbit in the plane x-y, circular, the velocity along +y
	const double R[3] = {7.0e6, 0.0, 0.0};
	const double V[3] = {0.0, 7500.0, 0.0};
	Mat3 lvlh;
	CHECK( LVLHFrame( R, V, lvlh ) );
	CHECK_NEAR( lvlh.m[0][1], 1.0, 1e-12 );// X along the velocity
	CHECK_NEAR( lvlh.m[2][0], -1.0, 1e-12 );// Z toward the center of the Earth
	CHECK_NEAR( lvlh.m[1][2], -1.0, 1e-12 );// Y against the angular momentum
	// the body in the LVLH attitude (nose along the velocity, belly to the Earth) is 0, 0, 0 in LVLH, and in inertial it is not
	Angles( Relative( lvlh, lvlh ), r, p, y );
	CHECK_NEAR( r, 0.0, 1e-9 );
	CHECK_NEAR( p, 0.0, 1e-9 );
	CHECK_NEAR( y, 0.0, 1e-9 );
	// the same nose up by 10 degrees relative to the LVLH, wherever it is in the orbit
	for (double ang = 0.0; ang < 6.2; ang += 1.3)
	{
		const double R2[3] = {7.0e6 * std::cos( ang ), 7.0e6 * std::sin( ang ), 0.0};
		const double V2[3] = {-7500.0 * std::sin( ang ), 7500.0 * std::cos( ang ), 0.0};
		Mat3 f;
		CHECK( LVLHFrame( R2, V2, f ) );
		const Mat3 body = Mul( Transpose( ry( 10.0 * pi / 180.0 ) ), f );
		Angles( Relative( body, f ), r, p, y );
		CHECK_NEAR( p, 10.0, 1e-9 );
		CHECK_NEAR( r, 0.0, 1e-9 );
		CHECK_NEAR( y, 0.0, 1e-9 );
	}
	// parallel position and velocity: no frame
	{
		const double rr[3] = {1.0e6, 0, 0}, vv[3] = {1.0e3, 0, 0};
		Mat3 f;
		CHECK( !LVLHFrame( rr, vv, f ) );
	}

	// REF: 0, 0, 0 at the press, then the angles relative to that attitude
	{
		const Mat3 pressed = Transpose( Mul( rz( 50.0 * pi / 180.0 ), ry( 20.0 * pi / 180.0 ) ) );
		Angles( Relative( pressed, pressed ), r, p, y );
		CHECK_NEAR( r, 0.0, 1e-9 );
		CHECK_NEAR( p, 0.0, 1e-9 );
		CHECK_NEAR( y, 0.0, 1e-9 );
		// then a roll of 15 degrees about the body axis X of the reference
		const Mat3 now = Transpose( Mul( Mul( rz( 50.0 * pi / 180.0 ), ry( 20.0 * pi / 180.0 ) ), rx( 15.0 * pi / 180.0 ) ) );
		Angles( Relative( now, pressed ), r, p, y );
		CHECK_NEAR( r, 15.0, 1e-9 );
		CHECK_NEAR( p, 0.0, 1e-9 );
		CHECK_NEAR( y, 0.0, 1e-9 );
	}
}

static void test_flight_log( void )
{
	tm t = {};
	t.tm_year = 126; t.tm_mon = 9; t.tm_mday = 28; t.tm_hour = 18; t.tm_min = 5; t.tm_sec = 9;
	char name[160];
	flightlog::FileName( name, sizeof( name ), "SSV_OV:1", t );
	CHECK( strcmp( name, "SSV_FlightLogs\\SSV_OV_1_20261028_180509.log" ) == 0 );// the characters that a file name can not have are replaced
	flightlog::FileName( name, sizeof( name ), "", t );
	CHECK( strcmp( name, "SSV_FlightLogs\\_20261028_180509.log" ) == 0 );
	// the option is on by default and is in the configuration file of the vehicle (test of the table of the options)
	CHECK( settings::ITEMS[settings::FLIGHT_DATA_LOG].def == 1.0 );
}

static void test_rcs_reduction( void )
{
	using namespace rcsmath;

	// the thrust is divided by the time acceleration (1 in real time or slower): the impulse of a step does not depend on the acceleration
	CHECK_NEAR( ThrustReduction( 1.0 ), 1.0, 1e-12 );
	CHECK_NEAR( ThrustReduction( 0.5 ), 1.0, 1e-12 );
	CHECK_NEAR( ThrustReduction( 0.0 ), 1.0, 1e-12 );
	CHECK_NEAR( ThrustReduction( 10.0 ), 10.0, 1e-12 );
	CHECK_NEAR( ThrustReduction( 1000.0 ), 1000.0, 1e-12 );
	for (double acc = 1.0; acc <= 10000.0; acc *= 10.0)
	{
		const double step = acc / 30.0;
		CHECK_NEAR( step / ThrustReduction( acc ), 1.0 / 30.0, 1e-9 );
	}

	// on rails from the time acceleration of the option (0 or 1 = never)
	CHECK( !OnRails( 1000.0, 0.0 ) );
	CHECK( !OnRails( 1000.0, 1.0 ) );
	CHECK( !OnRails( 999.0, 1000.0 ) );
	CHECK( OnRails( 1000.0, 1000.0 ) );
	CHECK( OnRails( 10000.0, 1000.0 ) );
	// mean propellant use of the hold: the figures of the JSC planning (0.7 lb/hr in LVLH, 3.6 lb/hr inertial)
	CHECK_NEAR( RailsPropellantRate( true ), 0.7 * 0.4536, 0.02 );
	CHECK_NEAR( RailsPropellantRate( false ), 3.6 * 0.4536, 0.05 );

	// the torque of the vernier jets in the DAP: the true one of the 2 jets (not a tenth of the primary torque), smaller than the primary by more than 50 times
	for (int axis = 0; axis < 3; axis++)
	{
		const double t = rcsjets::VernierTorque( axis, _V( 0.0, 0.0, 0.0 ) );
		const double ratio = rcsjets::VernierTorqueRatio( axis, _V( 0.0, 0.0, 0.0 ) );
		CHECK( t > 100.0 );
		CHECK( ratio < 0.02 );
		if (axis != 2) CHECK( (t > 2000.0) && (t < 6000.0) );// 2 jets of 24 lbf with arms of 20 m
	}
}

static void test_hyd_quantity( void )
{
	using namespace hydmodel;

	// the meter HYDRAULIC QUANTITY (COM 4.1): green from 40 to 80%, red above 95% and below 40%
	State s = InitialState();
	CHECK_NEAR( Quantity( s ), QTY_INITIAL, 1e-12 );// at the ambient temperature
	CHECK( (Quantity( s ) > 0.40) && (Quantity( s ) < 0.80) );

	// the fluid expands with the heat: dV = V * beta * dT, the reservoir takes it (the whole system holds about three reservoirs of fluid)
	CHECK_NEAR( QTY_PER_F, FLUID_EXPANSION * SYSTEM_VOLUME_RATIO, 1e-15 );
	s.hyd = AMBIENT + 100.0;
	CHECK_NEAR( Quantity( s ), QTY_INITIAL + (100.0 * FLUID_EXPANSION * SYSTEM_VOLUME_RATIO), 1e-12 );
	CHECK( Quantity( s ) > QTY_INITIAL );
	// the normal range of the temperature of the fluid (up to the spray of the boiler at 208 F) stays in the green range of the meter
	s.hyd = HYD_SETPOINT;
	CHECK( Quantity( s ) < 0.80 );
	s.hyd = 0.0;// cold: the fluid contracts
	CHECK( Quantity( s ) < QTY_INITIAL );
	CHECK( Quantity( s ) > 0.40 );
	// the quantity is never negative
	s.hydQty = 0.0;
	s.hyd = -200.0;
	CHECK_NEAR( Quantity( s ), 0.0, 1e-12 );

	// a leak takes from the volume (not from the expansion): the system with the fluid warm and leaking
	State w = InitialState();
	w.hyd = 200.0;
	w.hydLeak = 1.0 / 900.0;
	const double before = Quantity( w );
	Input in;
	in.speed = 0.0;
	in.env = AMBIENT;
	for (int t = 0; t < 60; t++) Step( w, in, 1.0 );
	CHECK( w.hydQty < QTY_INITIAL );
	CHECK( Quantity( w ) < before );
	// the pump can hold the pressure while there is fluid, even when it is cold
	State c = InitialState();
	c.hyd = -20.0;
	CHECK( HydraulicsOk( c ) );
	c.hydQty = 0.01;
	CHECK( !HydraulicsOk( c ) );
}

static void test_emergency_lighting( void )
{
	using namespace lightmath;

	// the emergency system puts its fixtures at full bright, whatever their controls say; without it nothing changes
	CHECK_NEAR( EmergencyLevel( 0.0, true ), 1.0, 1e-12 );
	CHECK_NEAR( EmergencyLevel( 0.3, true ), 1.0, 1e-12 );
	CHECK_NEAR( EmergencyLevel( 0.3, false ), 0.3, 1e-12 );
	CHECK_NEAR( EmergencyLevel( 0.0, false ), 0.0, 1e-12 );

	// the panels follow the cabin: the option is their glow with the lamps off, with the cabin lit they are as in the meshes
	CHECK_NEAR( PanelGlow( 0.5, 0.0 ), 0.5, 1e-12 );
	CHECK_NEAR( PanelGlow( 0.5, 1.0 ), 1.0, 1e-12 );
	CHECK_NEAR( PanelGlow( 0.5, 0.5 ), 0.75, 1e-12 );
	CHECK_NEAR( PanelGlow( 0.0, 0.0 ), 0.0, 1e-12 );
	CHECK_NEAR( PanelGlow( 1.0, 0.0 ), 1.0, 1e-12 );
	CHECK_NEAR( PanelGlow( -1.0, 3.0 ), 1.0, 1e-12 );
	CHECK_NEAR( PanelGlow( 2.0, -3.0 ), 1.0, 1e-12 );

	// the middeck floodlights
	CHECK_NEAR( MiddeckLevel( 0 ), 0.0, 1e-12 );
	CHECK_NEAR( MiddeckLevel( 1 ), 0.5, 1e-12 );
	CHECK_NEAR( MiddeckLevel( 2 ), 1.0, 1e-12 );
	CHECK_NEAR( MiddeckLevel( 7 ), 1.0, 1e-12 );
	CHECK_NEAR( MiddeckLevel( -1 ), 0.0, 1e-12 );
}

static void test_arm_links( void )
{
	using namespace massprops;

	// a rod: m L^2 / 12 about the axes perpendicular to it, nothing about its own axis
	{
		const VECTOR3 I = RodInertia( 12.0, _V( 0.0, 0.0, 0.0 ), _V( 2.0, 0.0, 0.0 ) );
		CHECK_NEAR( I.x, 0.0, 1e-12 );
		CHECK_NEAR( I.y, 4.0, 1e-12 );
		CHECK_NEAR( I.z, 4.0, 1e-12 );
		const VECTOR3 D = RodInertia( 12.0, _V( 0.0, 0.0, 0.0 ), _V( 1.0, 1.0, 0.0 ) );// diagonal: the share of each axis
		CHECK_NEAR( D.x, 1.0, 1e-12 );
		CHECK_NEAR( D.y, 1.0, 1e-12 );
		CHECK_NEAR( D.z, 2.0, 1e-12 );
	}

	// the arm stretched along x: no inertia about its own axis, the same about the other two
	const VECTOR3 sp = _V( 0.0, 0.0, 0.0 ), ep = _V( 6.4, 0.0, 0.0 ), wp = _V( 13.4, 0.0, 0.0 ), ee = _V( 15.0, 0.0, 0.0 );
	const VECTOR3 I = ArmInertia( sp, ep, wp, ee );
	CHECK_NEAR( I.x, 0.0, 1e-9 );
	CHECK( I.y > 1000.0 );
	CHECK_NEAR( I.y, I.z, 1e-9 );
	// folded, the arm has less inertia about the axis across the folded links than stretched
	const VECTOR3 Ifolded = ArmInertia( sp, _V( 6.4, 0.0, 0.0 ), _V( 0.0, 0.0, 0.0 ), _V( 1.6, 0.0, 0.0 ) );
	CHECK( Ifolded.y < I.y );

	// the arm as one body with this inertia is the same as its five pieces
	{
		const VECTOR3 hullPMI = _V( 50.0, 60.0, 70.0 );
		const double hullMass = 70000.0;
		const VECTOR3 hullCoG = _V( 0.0, 0.0, 0.0 );
		const VECTOR3 com = ArmCoM( sp, ep, wp, ee );
		const Point one = {ARM_MASS, com, I};
		const Point pieces[5] = {
			{ARM_MASS * SHARE_SHOULDER, sp, _V( 0.0, 0.0, 0.0 )},
			{ARM_MASS * SHARE_UPPER_BOOM, (sp + ep) * 0.5, RodInertia( ARM_MASS * SHARE_UPPER_BOOM, sp, ep )},
			{ARM_MASS * SHARE_ELBOW, ep, _V( 0.0, 0.0, 0.0 )},
			{ARM_MASS * SHARE_LOWER_BOOM, (ep + wp) * 0.5, RodInertia( ARM_MASS * SHARE_LOWER_BOOM, ep, wp )},
			{ARM_MASS * SHARE_WRIST, (wp + ee) * 0.5, RodInertia( ARM_MASS * SHARE_WRIST, wp, ee )}
		};
		VECTOR3 cog1, cog5;
		double mass1, mass5;
		const VECTOR3 pmi1 = ComposePMI( hullPMI, hullMass, hullCoG, &one, 1, cog1, mass1 );
		const VECTOR3 pmi5 = ComposePMI( hullPMI, hullMass, hullCoG, pieces, 5, cog5, mass5 );
		CHECK_NEAR( mass1, mass5, 1e-9 );
		CHECK_NEAR( pmi1.x, pmi5.x, 1e-9 );
		CHECK_NEAR( pmi1.y, pmi5.y, 1e-9 );
		CHECK_NEAR( pmi1.z, pmi5.z, 1e-9 );

		// the arm and the same arm taken away (negative mass and inertia) leave the vessel as it was
		const Point gone = {-ARM_MASS, com, _V( -I.x, -I.y, -I.z )};
		const Point both[2] = {one, gone};
		VECTOR3 cog2;
		double mass2;
		const VECTOR3 pmi2 = ComposePMI( hullPMI, hullMass, hullCoG, both, 2, cog2, mass2 );
		CHECK_NEAR( pmi2.x, hullPMI.x, 1e-9 );
		CHECK_NEAR( pmi2.y, hullPMI.y, 1e-9 );
		CHECK_NEAR( pmi2.z, hullPMI.z, 1e-9 );
		const VECTOR3 added = AddedPMI( both, 2, hullCoG, hullMass );
		CHECK_NEAR( added.x + added.y + added.z, 0.0, 1e-9 );
	}
}

static void test_controller_assign( void )
{
	// every input of a controller is written so it is read back as the same
	for (int type = 0; type < 3; type++)
	{
		const int ids[] = {0, 1, 5, 7};
		for (int id : ids)
		{
			int t = -1, i = -1;
			const std::string text = ctrlmath::FormatSubDevice( type, id );
			CHECK( ctrlmath::ParseSubDevice( text, t, i ) );
			CHECK( t == type );
			CHECK( i == id );
		}
	}

	// a line written for the file is read back by the loader
	const std::string line = ctrlmath::FormatConfigLine( "RHC_PITCH", ctrlmath::SUB_AXIS, ctrlmath::AX_RY, true, 0.05, 0.25, -0.125 );
	const std::vector<std::string> col = ctrlmath::SplitColumns( line );
	CHECK( col.size() == 6 );
	if (col.size() == 6)
	{
		int t, i;
		CHECK( col[0] == "RHC_PITCH" );
		CHECK( ctrlmath::ParseSubDevice( col[1], t, i ) && (t == ctrlmath::SUB_AXIS) && (i == ctrlmath::AX_RY) );
		CHECK( col[2] == "true" );
		CHECK_NEAR( atof( col[3].c_str() ), 0.05, 1e-9 );
		CHECK_NEAR( atof( col[4].c_str() ), 0.25, 1e-9 );
		CHECK_NEAR( atof( col[5].c_str() ), -0.125, 1e-9 );
	}

	// a button of a panel
	std::string pnl, comp;
	int act;
	const std::string pl = ctrlmath::FormatConfigLine( std::string( "PANEL A8A1/BRAKES:" ) + ctrlmath::ActionName( ctrlmath::ACT_HOLD_DOWN ), ctrlmath::SUB_BUTTON, 12, false, 0.0, 0.0, 0.0 );
	const std::vector<std::string> pc = ctrlmath::SplitColumns( pl );
	CHECK( pc.size() == 3 );
	CHECK( ctrlmath::ParsePanelInput( pc[0], pnl, comp, act ) && (pnl == "A8A1") && (comp == "BRAKES") && (act == ctrlmath::ACT_HOLD_DOWN) );

	// detection of the axis that is moved
	const double before[8] = {0.0, 0.0, -1.0, 0.1, 0.0, 0.0, 0.0, 0.0};
	double now[8] = {0.02, 0.1, -1.0, 0.1, 0.0, 0.0, 0.0, 0.0};
	CHECK( ctrlmath::DetectAxisMove( before, now, 8, 0.5 ) == -1 );
	now[2] = 0.2;// a throttle moved from the end
	now[4] = -0.7;
	CHECK( ctrlmath::DetectAxisMove( before, now, 8, 0.5 ) == 2 );// the largest movement
	now[2] = -1.0;
	CHECK( ctrlmath::DetectAxisMove( before, now, 8, 0.5 ) == 4 );

	CHECK_NEAR( ctrlmath::DefaultDeadzone( "SBTC" ), 0.0, 1e-12 );
	CHECK_NEAR( ctrlmath::DefaultDeadzone( "RHC_ROLL" ), 0.05, 1e-12 );
}

static void test_dispersion( void )
{
	// the tolerance of a part does not change, and depends on the seed and on the part
	CHECK( disp::Fixed( 12345, 7 ) == disp::Fixed( 12345, 7 ) );
	CHECK( disp::Fixed( 12345, 7 ) != disp::Fixed( 12346, 7 ) );
	CHECK( disp::Fixed( 12345, 7 ) != disp::Fixed( 12345, 8 ) );
	// the seeds that are near and the identifiers that are near are not alike
	int same = 0;
	for (uint32_t id = 0; id < 1000; id++) same += (fabs( disp::Fixed( 1, id ) - disp::Fixed( 2, id ) ) < 1e-6) ? 1 : 0;
	CHECK( same == 0 );

	// standard normal: mean 0, deviation 1 (limited to 3: 0.997)
	{
		const int n = 200000;
		double sum = 0.0, sum2 = 0.0, maxAbs = 0.0;
		for (int i = 0; i < n; i++)
		{
			const double z = disp::Fixed( 987654321ULL, static_cast<uint32_t>(i) );
			sum += z;
			sum2 += z * z;
			maxAbs = (std::max)( maxAbs, fabs( z ) );
		}
		const double mean = sum / n;
		const double sd = sqrt( (sum2 / n) - (mean * mean) );
		CHECK_NEAR( mean, 0.0, 0.01 );
		CHECK_NEAR( sd, 0.997, 0.005 );// the values above 3 are set to 3
		CHECK( maxAbs <= disp::TRUNCATION );
		CHECK( maxAbs > 2.8 );// the tails are there
	}

	// the noise: not the same value twice, the same sequence from the same seed, limited
	{
		disp::Noise a( 42 ), b( 42 ), c( 43 );
		bool allEqual = true, differs = false;
		double prev = 1e9;
		for (int i = 0; i < 1000; i++)
		{
			const double x = a.Normal(), y = b.Normal(), z = c.Normal();
			if (x != y) allEqual = false;
			if (x != z) differs = true;
			CHECK( fabs( x ) <= disp::TRUNCATION );
			CHECK( x != prev );
			prev = x;
		}
		CHECK( allEqual );
		CHECK( differs );
		for (int i = 0; i < 1000; i++)
		{
			const double u = a.Uniform();
			CHECK( (u > 0.0) && (u < 1.0) );
		}
	}

	// values with a dispersion
	CHECK_NEAR( disp::Rel( 100.0, 0.02, 1.5 ), 103.0, 1e-9 );
	CHECK_NEAR( disp::Rel( 100.0, 0.02, 1.5, 0.0 ), 100.0, 1e-12 );// scale 0: nominal
	CHECK_NEAR( disp::Abs( 3000.0, 30.0, -2.0, 0.5 ), 2970.0, 1e-9 );
	CHECK( disp::SeedFromText( "Discovery" ) == disp::SeedFromText( "Discovery" ) );
	CHECK( disp::SeedFromText( "Discovery" ) != disp::SeedFromText( "Atlantis" ) );
}


static void test_hyd_actuators( void )
{
	using namespace hydact;
	// power of a system
	CHECK_NEAR( Power( 3000.0 ), 1.0, 1e-12 );
	CHECK_NEAR( Power( 2800.0 ), 1.0, 1e-12 );
	CHECK_NEAR( Power( 900.0 ), 0.0, 1e-12 );
	CHECK_NEAR( Power( 1900.0 ), 0.5, 1e-12 );
	for (double p = 0.0; p < 3200.0; p += 50.0) CHECK( Power( p + 50.0 ) >= Power( p ) );

	// the switching valves of the elevons: the sources of DISP 87
	const double all[3] = {3000.0, 3000.0, 3000.0};
	CHECK( ElevonSystem( all, 0 ) == 2 );// left outboard: 3
	CHECK( ElevonSystem( all, 1 ) == 1 );// left inboard: 2
	CHECK( ElevonSystem( all, 2 ) == 2 );// right inboard: 3
	CHECK( ElevonSystem( all, 3 ) == 1 );// right outboard: 2
	const double no3[3] = {3000.0, 3000.0, 0.0};
	CHECK( ElevonSystem( no3, 0 ) == 0 );// the first standby: 1
	CHECK( ElevonSystem( no3, 2 ) == 0 );
	CHECK( ElevonSystem( no3, 1 ) == 1 );
	const double only2[3] = {0.0, 3000.0, 0.0};
	for (int e = 0; e < ELEVONS; e++) CHECK( ElevonSystem( only2, e ) == 1 );
	const double none[3] = {0.0, 500.0, 1200.0};
	for (int e = 0; e < ELEVONS; e++) CHECK( ElevonSystem( none, e ) == -1 );
	const double edge[3] = {1300.0, 1400.0, 1000.0};// the valve switches between 1200 and 1500
	CHECK( ElevonSystem( edge, 0 ) == 1 );// system 1 is below the switching pressure of 1350, the next source is 1 (index 0): 1300 no; index 1: 1400 yes
	CHECK( ElevonSystem( edge, 0, -100.0 ) == 0 );// a valve that switches at 1250 stays on the system 1 (1300)

	// rate: 20 deg/s, 13.9 deg/s with the four on the same system
	int sh[ELEVONS];
	ElevonSharing( all, nullptr, sh );
	for (int e = 0; e < ELEVONS; e++) CHECK( sh[e] == 2 );
	for (int e = 0; e < ELEVONS; e++) CHECK_NEAR( ElevonRate( sh[e] ), 20.0, 1e-12 );
	ElevonSharing( only2, nullptr, sh );
	for (int e = 0; e < ELEVONS; e++) CHECK( sh[e] == 4 );
	for (int e = 0; e < ELEVONS; e++) CHECK_NEAR( ElevonRate( sh[e] ), 13.9, 1e-12 );
	ElevonSharing( no3, nullptr, sh );
	for (int e = 0; e < ELEVONS; e++) CHECK( sh[e] == 2 );// the elevons of the lost system 3 go to the system 1, the others stay on the 2
	CHECK( ElevonRate( 3 ) < 20.0 );
	CHECK( ElevonRate( 3 ) > 13.9 );
	ElevonSharing( none, nullptr, sh );
	for (int e = 0; e < ELEVONS; e++) CHECK( sh[e] == 0 );

	// three motors
	const double full[3] = {1.0, 1.0, 1.0};
	const double two[3] = {1.0, 1.0, 0.0};
	const double one[3] = {0.0, 0.0, 1.0};
	const double zero[3] = {0.0, 0.0, 0.0};
	CHECK_NEAR( MotorDriveOutput( full ), 1.0, 1e-12 );
	CHECK_NEAR( MotorDriveOutput( two ), 0.8, 1e-12 );
	CHECK_NEAR( MotorDriveOutput( one ), 0.5, 1e-12 );// the manual: about half
	CHECK_NEAR( MotorDriveOutput( zero ), 0.0, 1e-12 );
	const double weak[3] = {1.0, 1.0, 0.5};// a system with a low pressure is between two and three motors
	CHECK( (MotorDriveOutput( weak ) > 0.8) && (MotorDriveOutput( weak ) < 1.0) );
	CHECK_NEAR( RSB_RATE_ONE_MOTOR / 0.5, 10.86, 0.01 );// full rate of the speedbrake

	// brakes: systems 1 and 2, the system 3 takes over one
	const double b123[3] = {3000.0, 3000.0, 3000.0};
	const double b12[3] = {3000.0, 3000.0, 0.0};
	const double b13[3] = {3000.0, 0.0, 3000.0};
	const double b3[3] = {0.0, 0.0, 3000.0};
	const double b1[3] = {3000.0, 0.0, 0.0};
	const double bn[3] = {0.0, 0.0, 0.0};
	CHECK_NEAR( BrakePower( b123 ), 1.0, 1e-12 );
	CHECK_NEAR( BrakePower( b12 ), 1.0, 1e-12 );
	CHECK_NEAR( BrakePower( b13 ), 1.0, 1e-12 );// the system 3 takes the place of the 2
	CHECK_NEAR( BrakePower( b3 ), 0.5, 1e-12 );// only one of the two lost systems is taken over
	CHECK_NEAR( BrakePower( b1 ), 0.5, 1e-12 );// the system 3 and one other lost: half
	CHECK_NEAR( BrakePower( bn ), 0.0, 1e-12 );
	// steering
	CHECK( SteeringPower( b123 ) );
	CHECK( SteeringPower( b13 ) );
	const double n2[3] = {0.0, 3000.0, 3000.0};
	CHECK( SteeringPower( n2 ) );
	const double n3[3] = {1200.0, 1000.0, 3000.0};
	CHECK( !SteeringPower( n3 ) );// caster mode

	// slew
	CHECK_NEAR( Slew( 0.0, 10.0, 3.0 ), 3.0, 1e-12 );
	CHECK_NEAR( Slew( 0.0, -10.0, 3.0 ), -3.0, 1e-12 );
	CHECK_NEAR( Slew( 0.0, 2.0, 3.0 ), 2.0, 1e-12 );

	// float with no pressure: toward the angle of the flow, faster with the dynamic pressure, never past it
	CHECK_NEAR( ElevonFloatAngle( 0.0 ), 0.0, 1e-12 );
	CHECK( ElevonFloatAngle( 30.0 ) < 0.0 );
	CHECK( ElevonFloatAngle( 200.0 ) >= FLOAT_MIN );
	double pos = 10.0;
	for (int i = 0; i < 2000; i++)
	{
		const double next = FloatStep( pos, -5.0, 3000.0, 0.1 );
		CHECK( next <= pos + 1e-12 );
		CHECK( next >= -5.0 - 1e-12 );
		pos = next;
	}
	CHECK_NEAR( pos, -5.0, 0.01 );
	CHECK( fabs( FloatStep( 10.0, 0.0, 5000.0, 1.0 ) ) < fabs( FloatStep( 10.0, 0.0, 500.0, 1.0 ) ) );// faster with more dynamic pressure
	// blowback: none with full power at moderate loads, all the load with no power
	CHECK_NEAR( BlowbackShare( 1.0, 3000.0, 10.0, 0.0 ), 0.0, 1e-12 );
	CHECK( BlowbackShare( 0.1, 8000.0, 20.0, 0.0 ) > 0.0 );
	CHECK( BlowbackShare( 0.0, 8000.0, 20.0, 0.0 ) > 0.99 );
	CHECK_NEAR( BlowbackShare( 0.0, 3000.0, 0.0, 0.0 ), 0.0, 1e-12 );// at the free angle there is no load
}


static void test_hyd_tolerances( void )
{
	using namespace hydmodel;
	disp::Mode off;
	disp::Mode on;
	on.tolerances = true;
	State a = InitialState();
	ApplyTolerances( a, 777, 0, off );
	CHECK_NEAR( a.tolOilHeat, 1.0, 1e-12 );
	CHECK_NEAR( a.tolHydCool, 1.0, 1e-12 );
	CHECK_NEAR( a.tolOilSet, 0.0, 1e-12 );
	ApplyTolerances( a, 777, 0, on );
	State b = InitialState();
	ApplyTolerances( b, 777, 1, on );
	CHECK( a.tolOilHeat != 1.0 );
	CHECK( a.tolOilHeat != b.tolOilHeat );// another system, other parts
	CHECK( (a.tolOilSet >= (-3.0 * SIGMA_OIL_SET)) && (a.tolOilSet <= (3.0 * SIGMA_OIL_SET)) );
	CHECK( (a.tolHydCool >= (1.0 - 3.0 * SIGMA_COOL)) && (a.tolHydCool <= (1.0 + 3.0 * SIGMA_COOL)) );
	State c = InitialState();
	ApplyTolerances( c, 777, 0, on );
	CHECK( c.tolHydHeat == a.tolHydHeat );// the same vehicle, the same parts
	disp::Mode zero = on;
	zero.scale = 0.0;
	ApplyTolerances( c, 777, 0, zero );
	CHECK_NEAR( c.tolHydHeat, 1.0, 1e-12 );
	CHECK_NEAR( c.tolHydSet, 0.0, 1e-12 );

	// the spray starts at the set point of this system, and cools with the flow of its bars
	Input in;
	in.speed = 1.0;
	in.pumpNorm = true;
	in.load = 1.0;
	in.ambient = AMBIENT;
	in.powered = true;
	in.controller = 1;
	in.n2Open = true;
	State nom = InitialState(), hot = InitialState(), strong = InitialState();
	nom.hyd = hot.hyd = strong.hyd = 215.0;
	nom.oil = hot.oil = strong.oil = 255.0;
	nom.through = hot.through = strong.through = true;
	hot.tolHydSet = 6.0;// the set point is higher: less spray at the same temperature
	strong.tolHydCool = 1.3;
	Step( nom, in, 0.1 );
	Step( hot, in, 0.1 );
	Step( strong, in, 0.1 );
	CHECK( hot.hydDuty < nom.hydDuty );
	CHECK_NEAR( strong.hydDuty, nom.hydDuty, 1e-12 );
	CHECK( (215.0 - strong.hyd) > (215.0 - nom.hyd) );// more cooling with more flow
	CHECK( (strong.water < nom.water) );// and more water used
}


static std::string jet_names( uint64_t set )
{
	std::string r;
	for (int i = 0; i < rcsjets::COUNT; i++)
	{
		if (set & rcsjets::Bit( i )) r += std::string( rcsjets::Jets()[i].name ) + " ";
	}
	return r;
}

static void test_rcs_jets( void )
{
	using namespace rcsjets;
	const int ROLL_AXIS = ROT_ROLL;
	const VECTOR3 cg = _V( 0.0, 0.0, 0.0 );
	CHECK( COUNT == 44 );
	int primary = 0, vernier = 0, fwd = 0;
	for (int i = 0; i < COUNT; i++)
	{
		primary += Jets()[i].vernier ? 0 : 1;
		vernier += Jets()[i].vernier ? 1 : 0;
		fwd += (Jets()[i].tank == TANK_FWD) ? 1 : 0;
		CHECK( (Len3( Jets()[i].dir ) > 0.9) && (Len3( Jets()[i].dir ) < 1.1) );// not exactly unit vectors in the data
		CHECK( Jets()[i].thrust > 90.0 );
		CHECK( Find( Jets()[i].name ) == i );
	}
	CHECK( primary == PRIMARY_COUNT );
	CHECK( vernier == 6 );
	CHECK( fwd == 16 );// 14 primary and 2 vernier
	CHECK( Find( "L2U" ) >= 0 );
	CHECK( Find( "XXX" ) == -1 );
	CHECK_NEAR( Jets()[Find( "F1F" )].thrust, 887.9 * 4.4482216, 5.0 );
	CHECK_NEAR( Jets()[Find( "L5D" )].thrust, 24.0 * 4.4482216, 1.0 );

	// rotations: the set turns about the axis, with little force and little moment about the other axes
	const char* axisName[3] = {"pitch", "yaw", "roll"};
	for (int axis = 0; axis < 3; axis++)
	{
		for (int sign = -1; sign <= 1; sign += 2)
		{
			for (int vern = 0; vern < 2; vern++)
			{
				const int k = vern ? 2 : 8;
				const uint64_t set = SelectRotation( axis, sign, k, vern != 0, 0, cg );
				CHECK( (Count( set ) >= 1) && (Count( set ) <= k) );
				VECTOR3 f, m;
				SetWrench( set, cg, f, m );
				const VECTOR3 a = RotationAxis( axis, sign );
				const double along = Dot3( m, a );
				CHECK( along > 0.0 );
				const VECTOR3 perp = m - (a * along);
				if (g_verbose) printf( "%s %+d %s: %s| along %.0f perp %.0f force %.0f\n", axisName[axis], sign, vern ? "vern" : "pri", jet_names( set ).c_str(), along, Len3( perp ), Len3( f ) );
				if (axis != ROLL_AXIS) CHECK( Len3( perp ) < ((vern ? 0.6 : 0.35) * along) );// the couple is clean (the roll is coupled with the others in the real vehicle too)
				CHECK( Len3( f ) < (0.5 * Jets()[0].thrust * k) );// the forces cancel (as far as the geometry allows)
				// the vernier jets are in the set of the vernier mode only
				for (int i = 0; i < COUNT; i++)
				{
					if (set & Bit( i )) CHECK( Jets()[i].vernier == (vern != 0) );
				}
			}
		}
	}
	// a jet that does not work is not selected, and the set still turns about the axis
	{
		const uint64_t all = SelectRotation( ROT_PITCH, 1, 8, false, 0, cg );
		const uint64_t failed = all & (~all + 1);// the lowest jet of the set
		const uint64_t other = SelectRotation( ROT_PITCH, 1, 8, false, failed, cg );
		CHECK( (other & failed) == 0 );
		CHECK( (Count( other ) >= 4) && (Count( other ) <= 8) );
	}
	// only the aft jets
	{
		const uint64_t aft = SelectRotation( ROT_PITCH, 1, 4, false, RegionMask( true ), cg );
		CHECK( (Count( aft ) >= 2) && (Count( aft ) <= 4) );
		CHECK( (aft & RegionMask( true )) == 0 );
	}
	// translations: force along the axis with little moment
	const char* trName[3] = {"fwd", "right", "up"};
	for (int axis = 0; axis < 3; axis++)
	{
		for (int sign = -1; sign <= 1; sign += 2)
		{
			const uint64_t set = SelectTranslation( axis, sign, 0, cg );
			CHECK( Count( set ) >= 2 );
			VECTOR3 f, m;
			SetWrench( set, cg, f, m );
			const VECTOR3 a = TranslationAxis( axis, sign );
			const double along = Dot3( f, a );
			if (g_verbose) printf( "trans %s %+d: %s| force %.0f of %.0f, moment %.0f\n", trName[axis], sign, jet_names( set ).c_str(), along, Len3( f ), Len3( m ) );
			CHECK( along > 3000.0 );
			CHECK( fabs( along - Len3( f ) ) < (0.1 * along) );// along the axis
			CHECK( Len3( m ) < (0.35 * along * LEVER) );
			for (int i = 0; i < COUNT; i++)
			{
				if (set & Bit( i )) CHECK( !Jets()[i].vernier );
			}
		}
	}
	// the vernier jets have a tenth of the moment or less
	for (int axis = 0; axis < 3; axis++)
	{
		const double r = VernierTorqueRatio( axis, cg );
		CHECK( (r > 0.0) && (r < 0.1) );
	}

	// the valve: delays and minimum on-time
	{
		Valve v;
		double on = 0.0;
		const double dt = 0.001;
		// a command of 20 ms lasts at least the 80 ms (plus the delays of the valve)
		for (int i = 0; i < 1000; i++)
		{
			on += StepValve( v, (i < 20), dt, PRIMARY_DELAY_ON, PRIMARY_DELAY_OFF ) * dt;
		}
		CHECK_NEAR( on, MIN_ON_TIME - PRIMARY_DELAY_ON + PRIMARY_DELAY_OFF, 0.004 );// the command lasts 80 ms, the valve is open that time less the delay on, plus the delay off
		// a long command: its length (the delays on and off almost cancel each other)
		Valve w;
		double on2 = 0.0;
		for (int i = 0; i < 1000; i++) on2 += StepValve( w, (i < 400), dt, PRIMARY_DELAY_ON, PRIMARY_DELAY_OFF ) * dt;
		CHECK_NEAR( on2, 0.400 - PRIMARY_DELAY_ON + PRIMARY_DELAY_OFF, 0.004 );
		// the thrust is not on before the delay
		Valve z;
		double first = 0.0;
		for (int i = 0; i < 30; i++) first += StepValve( z, true, dt, PRIMARY_DELAY_ON, PRIMARY_DELAY_OFF );
		CHECK( first < 1.0 );
		// with a long step (time acceleration) the share is between 0 and 1
		Valve y;
		const double share = StepValve( y, true, 0.5, PRIMARY_DELAY_ON, PRIMARY_DELAY_OFF );
		CHECK( (share > 0.9) && (share < 1.0) );
	}

	// installation of the jets: tolerances change the direction a little, and the same seed gives the same
	disp::Mode on;
	on.tolerances = true;
	disp::Mode off;
	const int jet = Find( "L2U" );
	const VECTOR3 d0 = InstalledDirection( jet, 5, off );
	const VECTOR3 d1 = InstalledDirection( jet, 5, on );
	const VECTOR3 d2 = InstalledDirection( jet, 5, on );
	CHECK_NEAR( Len3( d1 ), 1.0, 1e-9 );
	CHECK( Len3( d1 - d0 ) > 0.0 );
	CHECK( Len3( d1 - d0 ) < (3.0 * 3.0 * SIGMA_ANGLE * 0.01745 * 1.5) );// at most 3 sigma about two axes
	CHECK( Len3( d1 - d2 ) == 0.0 );
	CHECK( Len3( InstalledPosition( jet, 5, on ) - Jets()[jet].pos ) < 0.05 );
	CHECK( Len3( InstalledPosition( jet, 5, off ) - Jets()[jet].pos ) == 0.0 );
}


/**
 * Simulation of the orbiter with the real jets: rigid body (mass and moments of inertia of an orbiter with the payload bay empty), the 44 jets
 * with their valves, the selection of the jets of the vessel (rcsjets::SelectRotation and SelectTranslation) and a phase plane controller that
 * does what the orbit DAP does (rate limits, level 1/3, 2/3, 1 by the size of the error, vernier mode with the level 0.1). It tests how the
 * jets and the control behave as a whole (the real DAP needs the Orbiter and is not in this test).
 */
struct RcsSim
{
	const double mass = 100000.0;// [kg]
	const double inertia[3] = {8.7e6, 9.0e6, 1.1e6};// [kg m^2] about x (pitch), y (yaw), z (roll)
	double theta[3] = {0.0, 0.0, 0.0};// attitude about the axes of the DAP (pitch up, yaw right, roll right) [deg]
	double rate[3] = {0.0, 0.0, 0.0};// [deg/s] the same axes
	double velocity[3] = {0.0, 0.0, 0.0};// [m/s] in the frame of the vessel
	double fuel = 0.0;// [kg]
	double jetSeconds = 0.0;
	double time = 0.0;
	rcsjets::Valve valve[rcsjets::COUNT];
	uint64_t failed = 0;
	bool tolerances = false;
	bool noise = false;
	uint64_t seed = 12345;
	disp::Noise gen;
	double tolThrust[rcsjets::COUNT], tolDelay[rcsjets::COUNT], fireThrust[rcsjets::COUNT], fireDelay[rcsjets::COUNT];
	std::map<std::tuple<int, int, int, int, uint64_t>, uint64_t> cache;
	double cmdRot[3] = {0.0, 0.0, 0.0};// level of the DAP with sign
	double trackRate[3] = {0.0, 0.0, 0.0};// [deg/s] rate that the attitude follows (LVLH: the orbital rate)
	double disturb[3] = {0.0, 0.0, 0.0};// [deg/s^2] angular acceleration of the environment (gravity gradient)
	int cmdTrans[3] = {0, 0, 0};// forward, right, down

	RcsSim( void )
	{
		for (int j = 0; j < rcsjets::COUNT; j++) tolThrust[j] = tolDelay[j] = fireThrust[j] = fireDelay[j] = 1.0;
	}

	void SetTolerances( bool tol, bool noiseOn )
	{
		tolerances = tol;
		noise = noiseOn;
		for (int j = 0; j < rcsjets::COUNT; j++)
		{
			const uint32_t b = disp::id::RCS_JET + (16u * (uint32_t)j);
			tolThrust[j] = tol ? disp::Rel( 1.0, rcsjets::SIGMA_THRUST, disp::Fixed( seed, b + rcsjets::TOL_THRUST ) ) : 1.0;
			tolDelay[j] = tol ? disp::Rel( 1.0, rcsjets::SIGMA_DELAY, disp::Fixed( seed, b + rcsjets::TOL_DELAY ) ) : 1.0;
		}
	}

	uint64_t RotationSet( int axis, int sign, int k, bool vernier )
	{
		const std::tuple<int, int, int, int, uint64_t> key( axis, sign, k, vernier ? 1 : 0, failed );
		const auto it = cache.find( key );
		if (it != cache.end()) return it->second;
		return cache[key] = rcsjets::SelectRotation( axis, sign, k, vernier, failed, _V( 0.0, 0.0, 0.0 ) );
	}

	uint64_t TranslationSet( int axis, int sign )
	{
		const std::tuple<int, int, int, int, uint64_t> key( axis + 10, sign, 0, 0, failed );
		const auto it = cache.find( key );
		if (it != cache.end()) return it->second;
		return cache[key] = rcsjets::SelectTranslation( axis, sign, failed, _V( 0.0, 0.0, 0.0 ) );
	}

	/** One step: the commands select the jets, the valves open, the body moves. */
	void Step( double dt )
	{
		using namespace rcsjets;
		uint64_t on = 0;
		for (int i = 0; i < 3; i++)
		{
			if (fabs( cmdRot[i] ) > 0.0001)
			{
				bool vern;
				const int k = RotationJetCount( i, fabs( cmdRot[i] ), vern );
				on |= RotationSet( i, (cmdRot[i] > 0.0) ? 1 : -1, k, vern );
			}
			if (cmdTrans[i] != 0) on |= TranslationSet( i, (i == 2) ? -cmdTrans[i] : cmdTrans[i] );
		}
		VECTOR3 f = _V( 0.0, 0.0, 0.0 ), m = _V( 0.0, 0.0, 0.0 );
		for (int j = 0; j < COUNT; j++)
		{
			const bool command = ((on & Bit( j )) != 0) && ((failed & Bit( j )) == 0);
			if (command && !valve[j].requested && noise)
			{
				fireThrust[j] = disp::Rel( 1.0, NOISE_THRUST, gen.Normal() );
				fireDelay[j] = disp::Rel( 1.0, NOISE_DELAY, gen.Normal() );
			}
			const bool vern = Jets()[j].vernier;
			const double dly = tolDelay[j] * fireDelay[j];
			const double share = StepValve( valve[j], command, dt, (vern ? VERNIER_DELAY_ON : PRIMARY_DELAY_ON) * dly, (vern ? VERNIER_DELAY_OFF : PRIMARY_DELAY_OFF) * dly );
			if (share <= 0.0) continue;
			const double level = share * tolThrust[j] * fireThrust[j];
			f = f + (Force( j ) * level);
			m = m + (Moment( j, _V( 0.0, 0.0, 0.0 ) ) * level);
			fuel += level * Jets()[j].thrust * dt / ((vern ? VERNIER_ISP : PRIMARY_ISP) * 9.80665);
			jetSeconds += share * dt;
		}
		// rotation in the frame of the vessel; the axes of the DAP: pitch up = -x, yaw right = +y, roll right = -z
		const double w[3] = {-rate[0], rate[1], -rate[2]};
		const double alpha[3] = {m.x / inertia[0], m.y / inertia[1], m.z / inertia[2]};// [rad/s^2]
		for (int i = 0; i < 3; i++)
		{
			const double sgn = (i == 1) ? 1.0 : -1.0;
			const double nw = w[i] + (alpha[i] * DEG * dt) + (sgn * disturb[i] * dt);
			const double sign = (i == 1) ? 1.0 : -1.0;
			theta[i] += sign * 0.5 * (w[i] + nw) * dt;
			rate[i] = sign * nw;
		}
		velocity[0] += f.x / mass * dt;
		velocity[1] += f.y / mass * dt;
		velocity[2] += f.z / mass * dt;
		time += dt;
	}

	// phase plane controller of the DAP (the logic of OrbitDAP::RotationControl): rate error against a limit, level by the size of the error
	double attDb = 1.0, rateDb = 0.2, rateLimit = 0.1, maxRate = 0.5, gain = 0.25;
	bool vernierMode = false;
	void Control( const double* target )
	{
		for (int i = 0; i < 3; i++)
		{
			const double err = target[i] - theta[i];
			double wdes = trackRate[i] + (gain * err);
			wdes = (wdes > maxRate) ? maxRate : ((wdes < -maxRate) ? -maxRate : wdes);
			if ((fabs( err ) < attDb) && (fabs( rate[i] - trackRate[i] ) < rateDb) && !vernierMode) wdes = trackRate[i];
			const double rerr = wdes - rate[i];
			const double limit = vernierMode ? 0.02 : (((fabs( err ) < attDb) && (fabs( wdes - trackRate[i] ) < 1e-9)) ? 0.05 : rateLimit);
			if (fabs( rerr ) > limit)
			{
				double level = vernierMode ? 0.1 : 1.0;
				if (!vernierMode)
				{
					const double scale = fabs( rerr ) / limit;
					if (scale < 2.0) level = 1.0 / 3.0;
					else if (scale < 5.0) level = 0.667;
				}
				cmdRot[i] = (rerr > 0.0) ? level : -level;
			}
			else cmdRot[i] = 0.0;
		}
	}

	void Run( double seconds, const double* target, double dt = 0.04 )
	{
		const int n = static_cast<int>(seconds / dt);
		double moving[3] = {target[0], target[1], target[2]};
		for (int i = 0; i < n; i++)
		{
			for (int a = 0; a < 3; a++) moving[a] = target[a] + (trackRate[a] * time);
			Control( moving );
			Step( dt );
		}
	}
};

static void test_rcs_closed_loop( void )
{
	using namespace rcsjets;
	const double zero[3] = {0.0, 0.0, 0.0};
	const char* an[3] = {"pitch", "yaw", "roll"};

	// 1. hold: an error of 2 degrees in each axis is removed, the rates settle in the deadband and the fuel use is small
	for (int mode = 0; mode < 3; mode++)
	{
		RcsSim sim;
		if (mode == 1) sim.SetTolerances( true, false );// tolerances of the parts
		if (mode == 2) sim.SetTolerances( true, true );// and noise of each firing
		sim.theta[0] = 2.0;
		sim.theta[1] = -2.0;
		sim.theta[2] = 1.5;
		sim.Run( 240.0, zero );
		if (g_verbose) printf( "hold %d: err %.2f %.2f %.2f rate %.3f %.3f %.3f fuel %.2f kg, jet seconds %.1f, v %.3f %.3f %.3f\n", mode, sim.theta[0], sim.theta[1], sim.theta[2], sim.rate[0], sim.rate[1], sim.rate[2], sim.fuel, sim.jetSeconds, sim.velocity[0], sim.velocity[1], sim.velocity[2] );
		for (int i = 0; i < 3; i++)
		{
			CHECK( fabs( sim.theta[i] ) < 1.2 );
			CHECK( fabs( sim.rate[i] ) < 0.3 );
		}
		CHECK( sim.fuel < 25.0 );// 6 degrees of error removed with the minimum pulses of 80 ms (a better controller uses less: the DAP will be tuned)
		CHECK( sim.fuel > 0.1 );
	}

	// 2. the vernier mode: a small error removed with the vernier jets, with much less fuel than the primary mode
	{
		RcsSim pri, ver;
		pri.theta[0] = ver.theta[0] = 0.8;
		pri.attDb = ver.attDb = 0.2;
		ver.vernierMode = true;
		pri.Run( 240.0, zero );
		ver.Run( 240.0, zero );
		if (g_verbose) printf( "vernier: err %.3f rate %.4f fuel %.3f kg (primary %.3f, err %.3f)\n", ver.theta[0], ver.rate[0], ver.fuel, pri.fuel, pri.theta[0] );
		CHECK( fabs( ver.theta[0] ) < 0.35 );
		CHECK( ver.fuel < pri.fuel );
		for (int j = 0; j < COUNT; j++) CHECK( (ver.valve[j].commanded == false) || Jets()[j].vernier );// only vernier jets
	}

	// 3. a rotation of 5 degrees in each axis: the target is reached and the cross coupling (translation) stays small
	{
		RcsSim sim;
		const double target[3] = {5.0, -5.0, 5.0};
		sim.Run( 300.0, target );
		if (g_verbose) printf( "maneuver: theta %.2f %.2f %.2f rate %.3f %.3f %.3f fuel %.2f v %.3f %.3f %.3f\n", sim.theta[0], sim.theta[1], sim.theta[2], sim.rate[0], sim.rate[1], sim.rate[2], sim.fuel, sim.velocity[0], sim.velocity[1], sim.velocity[2] );
		for (int i = 0; i < 3; i++)
		{
			CHECK( fabs( sim.theta[i] - target[i] ) < 1.2 );
			CHECK( fabs( sim.rate[i] ) < 0.3 );
		}
		const double dv = sqrt( (sim.velocity[0] * sim.velocity[0]) + (sim.velocity[1] * sim.velocity[1]) + (sim.velocity[2] * sim.velocity[2]) );
		CHECK( dv < 0.5 );// the jets of the rotations cancel their forces (as far as the geometry allows)
	}

	// 4. every jet that works can fail: the control still holds (the jets are selected again)
	{
		RcsSim healthy;
		healthy.theta[0] = 3.0;
		healthy.Run( 240.0, zero );
		int controllable = 0;
		for (int j = 0; j < PRIMARY_COUNT + 6; j++)
		{
			if (Jets()[j].vernier) continue;
			RcsSim sim;
			sim.failed = Bit( j );
			sim.theta[0] = 3.0;
			sim.theta[1] = -3.0;
			sim.theta[2] = 2.0;
			sim.Run( 240.0, zero );
			bool ok = true;
			for (int i = 0; i < 3; i++) ok = ok && (fabs( sim.theta[i] ) < 1.5) && (fabs( sim.rate[i] ) < 0.35);
			controllable += ok ? 1 : 0;
			if (!ok && g_verbose) printf( "jet %s failed: err %.2f %.2f %.2f rate %.3f %.3f %.3f\n", Jets()[j].name, sim.theta[0], sim.theta[1], sim.theta[2], sim.rate[0], sim.rate[1], sim.rate[2] );
		}
		CHECK( controllable == PRIMARY_COUNT );// any single failure: still controllable
		// the pods lost: the forward jets alone cannot turn the vehicle about the three axes (not a failure of the test: the vehicle is lost)
		RcsSim aftOnly;
		aftOnly.failed = RegionMask( true ) & ~0ULL;
		aftOnly.theta[0] = 3.0;
		aftOnly.Run( 240.0, zero );
		CHECK( fabs( aftOnly.theta[0] ) < 1.5 );// the pods alone turn the vehicle in pitch
	}

	// 5. translation: the acceleration of each axis and the rotation that it causes (the DAP corrects it)
	for (int axis = 0; axis < 3; axis++)
	{
		RcsSim sim;
		sim.cmdTrans[axis] = 1;
		for (int i = 0; i < 25; i++) sim.Step( 0.04 );// 1 s
		sim.cmdTrans[axis] = 0;
		const double speed = fabs( sim.velocity[(axis == 0) ? 2 : ((axis == 1) ? 0 : 1)] );
		double wmax = 0.0;
		for (int i = 0; i < 3; i++) wmax = (std::max)( wmax, fabs( sim.rate[i] ) );
		if (g_verbose) printf( "trans %d: speed %.3f m/s after 1 s, rotation %.3f deg/s\n", axis, speed, wmax );
		CHECK( speed > 0.1 );// 0.1 m/s in 1 s with the force of the jets
		CHECK( speed < 0.5 );
		CHECK( wmax < 0.6 );// the rotation that a translation causes can be corrected
	}

	// 5b. attitude hold in LVLH, payload bay to Earth with the principal axes along the axes of the orbit (ZLV, Y-POP): the gravity gradient
	// has only the torque of the product of inertia (3 w^2 Ixz, about 1.5 Nm), plus a disturbance of the environment (venting, crew) of 10 Nm
	// that changes sign every 5 minutes. Real data (planning of the JSC, 1977, Table II): the vernier hold in LVLH uses 0.7 lb/hr at 100 nmi
	// (0.32 kg/hr); the inertial hold 3.6 lb/hr (1.6 kg/hr). The primary jets are not used for the hold (no figure published).
	for (int vern = 0; vern < 2; vern++)
	{
		RcsSim sim;
		sim.vernierMode = (vern != 0);
		sim.gain = vern ? 0.01 : 0.25;
		sim.trackRate[0] = 360.0 / 5400.0;
		const double gg = (3.0 * pow( 6.283185307 / 5400.0, 2 ) * 0.3e6 / sim.inertia[0]) * DEG;
		const double env = (10.0 / sim.inertia[0]) * DEG;
		double fuelAt = 0.0;
		for (int block = 0; block < 12; block++)
		{
			sim.disturb[0] = gg + ((block % 2) ? env : -env);
			sim.Run( 300.0, zero );
		}
		fuelAt = sim.fuel;
		if (g_verbose) printf( "LVLH hold %s, 1 hour: fuel %.2f kg (%.2f lb/hr; the real vernier hold 0.7 lb/hr), error %.2f deg, jet seconds %.1f\n", vern ? "vernier" : "primary", fuelAt, fuelAt / 0.4536, sim.theta[0] - (sim.trackRate[0] * sim.time), sim.jetSeconds );
		CHECK( fabs( sim.theta[0] - (sim.trackRate[0] * sim.time) ) < 1.5 );// it follows the orbit
		if (vern) CHECK( fuelAt < 1.0 );// within a factor 3 of the real 0.32 kg/hr: the DAP is tuned against flights
	}

	// 6. impulse of a single pulse: the minimum on-time, the fuel of one pulse of the jets is small
	{
		RcsSim sim;
		sim.cmdRot[0] = 1.0 / 3.0;
		sim.Step( 0.04 );
		sim.cmdRot[0] = 0.0;
		for (int i = 0; i < 25; i++) sim.Step( 0.04 );
		if (g_verbose) printf( "one pulse: rate %.3f deg/s, fuel %.3f kg\n", sim.rate[0], sim.fuel );
		CHECK( sim.rate[0] > 0.0 );// the minimum impulse of 80 ms lasts beyond the 40 ms of the command
		CHECK( sim.fuel > 0.01 );
		CHECK( sim.fuel < 1.0 );
	}
}


int main( int argc, char** argv )
{
	g_verbose = (getenv( "RCS_VERBOSE" ) != NULL);
	srand( 12345 );
	test_pdrs_logic();
	test_frames();
	test_euler();
	test_axis_angle();
	test_por_error();
	test_rotate_about_por();
	test_pointing();
	test_celestial();
	test_controller_axes();
	test_controller_pov();
	test_controller_config();
	test_load_scale();
	test_safing_and_brakes();
	test_apu_model();
	test_rms_reaction();
	test_rms_kin();
	test_hyd_model();
	test_settings( (argc > 1) ? argv[1] : NULL );
	test_free_move();
	test_lighting();
	test_panel_lighting_table();
	test_mass_props();
	test_controller_parsers();
	test_controller_assign();
	test_hyd_environment();
	test_arm_links();
	test_emergency_lighting();
	test_hyd_quantity();
	test_rcs_reduction();
	test_flight_log();
	test_adi();
	test_dispersion();
	test_hyd_actuators();
	test_hyd_tolerances();
	test_rcs_jets();
	test_rcs_closed_loop();
	test_controller_examples( (argc > 1) ? argv[1] : NULL );
	printf( "%d checks, %d failed\n", g_checks, g_failed );
	return (g_failed == 0) ? 0 : 1;
}
