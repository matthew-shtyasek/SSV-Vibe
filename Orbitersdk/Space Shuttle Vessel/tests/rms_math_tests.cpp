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
#include "../OV/APUModel.h"
#include "../OV/RMSReaction.h"
#include "../OV/ParameterValues.h"
#include "../OV/SettingsModel.h"
#include "../OV/FreeMoveMath.h"
#include "../OV/LightingMath.h"
#include "../OV/CabinLightingData.h"
#include "../OV/MassProps.h"
#include "../OV/HandControllers.h"
#include <fstream>
#include <string>
#include <vector>

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
	d.injector = 500.0;
	CHECK_NEAR( DetonationProbability( d ), 0.5, 1e-12 );
	d.injector = 700.0;
	CHECK_NEAR( DetonationProbability( d ), 1.0, 1e-12 );
	d = ColdThermal();
	d.pump = 240.0;
	CHECK_NEAR( DetonationProbability( d ), 0.5, 1e-12 );
	d = ColdThermal();
	d.valve = 230.0;
	CHECK_NEAR( DetonationProbability( d ), 0.5, 1e-12 );
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
	CHECK( !IsInside( _V( 0.6, 0.0, 0.0 ) ) );
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
		CHECK( (it.kind >= KIND_BOOL) && (it.kind <= KIND_FLOAT) );
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
		e.Move( -1 );// circular
		CHECK( e.Cursor() == (COUNT - 1) );
		e.Move( 1 );
		CHECK( e.Cursor() == 0 );
		for (int i = 0; i < COUNT; i++) e.Move( 1 );
		CHECK( e.Cursor() == 0 );

		// change and reset of an option
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
		for (int i = 0; i < COUNT; i++) CHECK_NEAR( e.Value( i ), ITEMS[i].def, 1e-12 );
		CHECK( !e.ResetAll() );

		// values from the options (limited to the range)
		double v[COUNT];
		for (int i = 0; i < COUNT; i++) v[i] = 1000.0;
		e.SetValues( v );
		for (int i = 0; i < COUNT; i++) CHECK_NEAR( e.Value( i ), ITEMS[i].max, 1e-12 );
		CHECK( !e.IsDirty() );

		// windows: the selected line is always shown
		for (int rows = 1; rows <= (COUNT + 3); rows++)
		{
			Editor w;
			for (int c = 0; c < COUNT; c++)
			{
				const int first = w.FirstRow( rows );
				CHECK( first >= 0 );
				CHECK( (rows >= COUNT) ? (first == 0) : ((first + rows) <= COUNT) );
				CHECK( (w.Cursor() >= first) && (w.Cursor() < (first + ((rows < COUNT) ? rows : COUNT))) );
				w.Move( 1 );
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
		for (int i = 1; i < COUNT; i++) groups += (ITEMS[i].group != ITEMS[i - 1].group) ? 1 : 0;
		for (int i = 0; i < groups; i++) c.MoveGroup( 1 );
		CHECK( c.Cursor() == 0 );
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

int main( int argc, char** argv )
{
	srand( 12345 );
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
	test_settings( (argc > 1) ? argv[1] : NULL );
	test_free_move();
	test_lighting();
	test_panel_lighting_table();
	test_mass_props();
	test_controller_parsers();
	test_controller_examples( (argc > 1) ? argv[1] : NULL );
	printf( "%d checks, %d failed\n", g_checks, g_failed );
	return (g_failed == 0) ? 0 : 1;
}
