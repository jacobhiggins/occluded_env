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
  DifferentialState left_bound;
  DifferentialState right_bound;
  DifferentialState upper_bound;
  // DifferentialState max_vel;
  // DifferentialState m_left; // Slope for robot-to-corner boundary on left side
  DifferentialState m_right;  // Slope for robot-to-corner boundary on right side
  DifferentialState dum1;
  DifferentialState dum2;


  Control ax;
  Control ay;
  Control epsilon;
  // Control epsilon_vel;

  DifferentialEquation f;


  f << dot(x) == vx;
  f << dot(y) == vy;
  f << dot(vx) == ax;
  f << dot(vy) == ay;
  
  f << dot(phi_right) == (vy/(x) - (((y+dum1)*vx)/pow(x,2)))/(1 + pow((y+dum1)/x,2))/(y+dum1) - atan((y+dum1)/(x))*vy/pow((y+dum1),2); // Phi over y
  // f << dot(phi_left) == (vy/(x - xc_left) - vx*(y-yc_left)/pow(x-xc_left,2))/(1 + pow((y-yc_left)/(x-xc_left),2))/(y-yc_left)
  //       - vy*atan((y-yc_left)/(x-xc_left))/pow(y-yc_left,2);

  // f << dot(xc_left) == 0;
  // f << dot(yc_left) == 0;

  f << dot(left_bound) == 0; // Location of left wall, not to hit
  f << dot(right_bound) == 0; // Location of right wall, not to hit
  f << dot(upper_bound) == 0; // Corner location when unsafe
  // f << dot(max_vel) == 0;
  // f << dot(m_left) == 0;
  f << dot(m_right) == 0;

  f << dot(dum1) == -1*epsilon;
  // f << dot(dum2) == epsilon_vel;

  // Reference funciton
  Function h, hN;
  h << x << y <<  phi_right << ax << ay << epsilon;
  hN << x << y << phi_right;

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
  ocp.subjectTo( pow(vx,2) + pow(vy,2) - 2 - epsilon<= 0);
  ocp.subjectTo( left_bound - x - epsilon <= 0 );
  ocp.subjectTo( x - epsilon - m_right*y - right_bound <= 0);
  ocp.subjectTo( y - upper_bound - epsilon <= 0 );
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

  if (mpc.exportCode( "acc_cmd_export" ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	mpc.printDimensionsQP( );

	return EXIT_SUCCESS;
}