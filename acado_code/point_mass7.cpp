#include <acado_code_generation.hpp>
#include <math.h>

int main()
{
  // int xc;
  // int yc;
  // xc = 15;
  // yc = 0;

  USING_NAMESPACE_ACADO

  DifferentialState x;
  DifferentialState y;
  DifferentialState vx;
  DifferentialState vy;
  DifferentialState ax;
  DifferentialState ay;
  DifferentialState phi_right;
  DifferentialState phi_left;
  DifferentialState xc_left;
  DifferentialState yc_left;
  DifferentialState left_bound;
  DifferentialState right_bound;
  DifferentialState m_left; // Slope for robot-to-corner boundary on left side
  DifferentialState m_right;  // Slope for robot-to-corner boundary on right side
  DifferentialState dum1;
  DifferentialState dum2;


  Control ax_dot;
  Control ay_dot;
  Control epsilon;

  DifferentialEquation f;


  f << dot(x) == vx;
  f << dot(y) == vy;
  f << dot(vx) == ax;
  f << dot(vy) == ay;
  f << dot(ax) == ax_dot;
  f << dot(ay) == ay_dot;
  
  f << dot(phi_right) == (vy/x - ((y*vx)/pow(x,2)))/(1 + pow(y/x,2))/y - atan(y/x)*vy/pow(y,2); // Phi over y
  f << dot(phi_left) == (vy/(x - xc_left) - vx*(y-yc_left)/pow(x-xc_left,2))/(1 + pow((y-yc_left)/(x-xc_left),2))/(y-yc_left)
        - vy*atan((y-yc_left)/(x-xc_left))/pow(y-yc_left,2);

  f << dot(xc_left) == 0;
  f << dot(yc_left) == 0;

  f << dot(left_bound) == 0; // Location of left wall, not to hit
  f << dot(right_bound) == 0; // Location of right wall, not to hit
  f << dot(m_left) == 0;
  f << dot(m_right) == 0;

  f << dot(dum1) == epsilon;

  // Reference funciton
  Function h, hN;
  h << x << y <<  phi_right << phi_left << ax_dot << ay_dot << epsilon;
  hN << x << y << phi_right << phi_left;

  // Weigting Matrices
  BMatrix W = eye<bool>( h.getDim() );
  BMatrix WN = eye<bool>( hN.getDim() );

  OCP ocp(0.0,5.0,50);

  ocp.subjectTo( f );

  ocp.minimizeLSQ( W, h );

  ocp.minimizeLSQEndTerm( WN, hN );

  ocp.subjectTo( -4 <= ax <= 4 );
  ocp.subjectTo( -4 <= ay <= 4 );
  ocp.subjectTo( -2 <= vx <= 2 );
  ocp.subjectTo( -2 <= vy <= 2 );
  ocp.subjectTo( left_bound - x - epsilon <= 0 );
  ocp.subjectTo( x - epsilon - m_right*y - right_bound <= 0);
  // ocp.subjectTo( m_left*(y-yc_left) - (x-xc_left) - epsilon <= 0 );
  ocp.subjectTo( 0 <= epsilon);
  // Now that the MPC is setup, export the code
  OCPexport mpc( ocp );

  mpc.set( DISCRETIZATION_TYPE,         MULTIPLE_SHOOTING );
  // NOTE: for a differential algebraic equation, need to use implicit RK
	// mpc.set( INTEGRATOR_TYPE,             INT_IRK_GL6         );
  mpc.set(INTEGRATOR_TYPE,              INT_IRK_GL4);
	mpc.set( NUM_INTEGRATOR_STEPS,        200              );

	mpc.set( QP_SOLVER,                   QP_QPOASES      );
// 	mpc.set( HOTSTART_QP,                 YES             );
// 	mpc.set( LEVENBERG_MARQUARDT,         1.0e-4          );
	mpc.set( GENERATE_TEST_FILE,          YES             );
	mpc.set( GENERATE_MAKE_FILE,          NO             );
	mpc.set( GENERATE_MATLAB_INTERFACE,   YES             );
	mpc.set( GENERATE_SIMULINK_INTERFACE, YES             );
  mpc.set( PRINTLEVEL ,                 DEBUG );
  mpc.set( CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);
  mpc.set( INFEASIBLE_QP_HANDLING, YES);

  if (mpc.exportCode( "point_mass_export7" ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	mpc.printDimensionsQP( );

	return EXIT_SUCCESS;
}


// ************************

// #include <acado_code_generation.hpp>
// #include <math.h>

// int main()
// {


//   USING_NAMESPACE_ACADO

//   DifferentialState x;
//   DifferentialState y;
//   DifferentialState vx;
//   DifferentialState vy;
//   DifferentialState ax;
//   DifferentialState ay;
//   DifferentialState phi_right;
//   DifferentialState phi_left;
//   DifferentialState left_bound;
//   DifferentialState right_bound;
//   DifferentialState xc_left;
//   DifferentialState yc_left;
//   DifferentialState m;  // Slope for robot-to-corner boundary
//   DifferentialState dummy1;
//   DifferentialState dummy2;

//   Control ax_dot;
//   Control ay_dot;
//   Control epsilon;
//   Control epsilon2; // For safety bound

//   DifferentialEquation f;


//   f << dot(x) == vx;
//   f << dot(y) == vy;
//   f << dot(vx) == ax;
//   f << dot(vy) == ay;

//   f << dot(ax) == ax_dot;
//   f << dot(ay) == ay_dot;
 
//   f << dot(phi_right) == (vy/x - ((y*vx)/pow(x,2)))/(1 + pow(y/x,2))/y - atan(y/x)*vy/pow(y,2); // Phi over y
//   f << dot(phi_left) == (vy/(x - xc_left) - vx*(y-yc_left)/pow(x-xc_left,2))/(1 + pow((y-yc_left)/(x-xc_left),2))/(y-yc_left)
//         - vy*atan((y-yc_left)/(x-xc_left))/pow(y-yc_left,2);

//   f << dot(left_bound) == 0; // Location of left wall, not to hit
//   f << dot(right_bound) == 0; // Location of right wall, not to hit
//   f << dot(xc_left) == 0;
//   f << dot(yc_left) == 0;
//   f << dot(m) == 0;
//   f << dot(dummy1) == epsilon;
//   f << dot(dummy2) == epsilon2;

//   // Reference funciton
//   Function h, hN;
//   h << x << y << phi_right << phi_left << ax_dot << ay_dot << epsilon << epsilon2;
//   hN << x << y << phi_right << phi_left;

//   // Weigting Matrices
//   BMatrix W = eye<bool>( h.getDim() );
//   BMatrix WN = eye<bool>( hN.getDim() );
//   // W(0,0) = 1;
//   // WN(0,0) = 1;
//   // W(2,2) = 100; // for good run, both weihts = 500
//   // WN(2,2) = 100;

//   // WN *= 5;

//   OCP ocp(0.0,5.0,50);

//   ocp.subjectTo( f );

//   ocp.minimizeLSQ( W, h );

//   ocp.minimizeLSQEndTerm( WN, hN );

//   ocp.subjectTo( -5 <= ax <= 5 );
//   ocp.subjectTo( -5 <= ay <= 5 );
//   ocp.subjectTo( -20 <= vx <= 20 );
//   ocp.subjectTo( -20 <= vy <= 20 );
//   ocp.subjectTo( left_bound - x - epsilon <= 0 );
//   ocp.subjectTo( x - epsilon - m*y - right_bound <= 0);
//   ocp.subjectTo( 0 <= epsilon);
//   ocp.subjectTo( 0 <= epsilon2);
//   // Now that the MPC is setup, export the code
//   OCPexport mpc( ocp );

//   mpc.set( DISCRETIZATION_TYPE,         MULTIPLE_SHOOTING );
//   // NOTE: for a differential algebraic equation, need to use implicit RK
// 	// mpc.set( INTEGRATOR_TYPE,             INT_IRK_GL6         );
//   mpc.set(INTEGRATOR_TYPE,              INT_IRK_GL4);
// 	mpc.set( NUM_INTEGRATOR_STEPS,        200              );

// 	mpc.set( QP_SOLVER,                   QP_QPOASES      );
// // 	mpc.set( HOTSTART_QP,                 YES             );
// // 	mpc.set( LEVENBERG_MARQUARDT,         1.0e-4          );
// 	mpc.set( GENERATE_TEST_FILE,          YES             );
// 	mpc.set( GENERATE_MAKE_FILE,          NO             );
// 	mpc.set( GENERATE_MATLAB_INTERFACE,   YES             );
// 	mpc.set( GENERATE_SIMULINK_INTERFACE, YES             );
//   mpc.set( PRINTLEVEL ,                 DEBUG );
//   mpc.set( CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);
//   mpc.set( INFEASIBLE_QP_HANDLING, YES);

//   if (mpc.exportCode( "point_mass_export7" ) != SUCCESSFUL_RETURN)
// 		exit( EXIT_FAILURE );

// 	mpc.printDimensionsQP( );

// 	return EXIT_SUCCESS;
// }