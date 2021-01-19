// 11/25/2020: Use a more local frame for MPC

#include <acado_code_generation.hpp>
#include <math.h>

int main()
{
  
  double x_offset = 0;

  USING_NAMESPACE_ACADO

  DifferentialState x;
  DifferentialState y;
  DifferentialState vx;
  DifferentialState vy;
  DifferentialState phi_right;
  // DifferentialState phi_left;
  // DifferentialState xc_left;
  // DifferentialState yc_left;
  DifferentialState left_m;
  DifferentialState left_x;
  DifferentialState left_y;
  DifferentialState right_m;
  DifferentialState right_x;
  DifferentialState right_y;
  DifferentialState right_sign;
  DifferentialState top;
  DifferentialState rightcorner_x;
  DifferentialState rightcorner_y;
  DifferentialState shift_corner_y;
  DifferentialState dum1;

  Control ax;
  Control ay;
  Control epsilon;

  DifferentialEquation f;


  f << dot(x) == vx;
  f << dot(y) == vy;
  f << dot(vx) == ax;
  f << dot(vy) == ay;
  
  f << dot(phi_right) == (vy/(x-rightcorner_x) - (((y-rightcorner_y-shift_corner_y)*vx)/pow(x-rightcorner_x,2)))/(1 + pow((y-rightcorner_y-shift_corner_y)/(x-rightcorner_x),2))/(y-rightcorner_y-shift_corner_y) - atan((y-rightcorner_y-shift_corner_y)/(x-rightcorner_x))*vy/pow((y-rightcorner_y-shift_corner_y),2); // Phi over y
  // f << dot(phi_right) == 0;
  // f << dot(phi_left) == (vy/(x - xc_left) - vx*(y-yc_left)/pow(x-xc_left,2))/(1 + pow((y-yc_left)/(x-xc_left),2))/(y-yc_left)
  //       - vy*atan((y-yc_left)/(x-xc_left))/pow(y-yc_left,2);

  // f << dot(xc_left) == 0;
  // f << dot(yc_left) == 0;

  f << dot(left_m) == 0; // Location of left wall, not to hit
  f << dot(left_x) == 0;
  f << dot(left_y) == 0;
  f << dot(right_m) == 0; // Location of right wall, not to hit
  f << dot(right_x) == 0;
  f << dot(right_y) == 0;
  f << dot(right_sign) == 0;
  f << dot(top) == 0;
  f << dot(rightcorner_x) == 0;
  f << dot(rightcorner_y) == 0;
  f << dot(shift_corner_y) == 0;

  f << dot(dum1) == -1*epsilon;

  // Reference funciton
  Function h, hN;
  h << x << y << phi_right << vx << vy << ax << ay << epsilon;
  hN << x << y;

  // Weigting Matrices
  BMatrix W = eye<bool>( h.getDim() );
  BMatrix WN = eye<bool>( hN.getDim() );

  OCP ocp(0.0,5.0,50);

  ocp.subjectTo( f );

  ocp.minimizeLSQ( W, h );

  ocp.minimizeLSQEndTerm( WN, hN );

  ocp.subjectTo( -1 <= ax <= 1 );
  ocp.subjectTo( -1 <= ay <= 1 );
  // ocp.subjectTo( -2 <= vx <= 2 );
  // ocp.subjectTo( -2 <= vy <= 2 );
  ocp.subjectTo( pow(vx,2) + pow(vy,2) - epsilon<= 2);
  ocp.subjectTo( (x - left_x) - left_m*(y - left_y) - epsilon >= 0 ); // Left constraint
  ocp.subjectTo( right_sign*((x - right_x) - right_m*(y - right_y) - epsilon) <= 0 ); // Right constraint
  ocp.subjectTo( y - top <= 0); // Top constraint
  // ocp.subjectTo( (x - right_x) - 1*pow((x-right_x),2) - epsilon<= 0 ); // NONLINEAR CONSTRAINT, CHANGE BACK
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

  if (mpc.exportCode( "acc_cmd_local_frame_export" ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	mpc.printDimensionsQP( );

	return EXIT_SUCCESS;
}