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

int main( void )
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
	printf( "%d checks, %d failed\n", g_checks, g_failed );
	return (g_failed == 0) ? 0 : 1;
}
