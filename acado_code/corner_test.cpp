#include <acado_code_generation.hpp>
#include <math.h>

int main()
{;

  USING_NAMESPACE_ACADO

  DifferentialState x;
  DifferentialState y;
  DifferentialState dum;

  Control vx;
  Control vy;
  Control epsilon_x;
  Control epsilon_y;
  Control epsilon_total;

  DifferentialEquation f;


  f << dot(x) == vx;
  f << dot(y) == vy;
  f << dot(dum) == epsilon_total*epsilon_x*epsilon_y;
  
//   f << dot(phi_right) == (vy/(x) - (((y+dum1)*vx)/pow(x,2)))/(1 + pow((y+dum1)/x,2))/(y+dum1) - atan((y+dum1)/(x))*vy/pow((y+dum1),2); // Phi over y
  // f << dot(phi_left) == (vy/(x - xc_left) - vx*(y-yc_left)/pow(x-xc_left,2))/(1 + pow((y-yc_left)/(x-xc_left),2))/(y-yc_left)
  //       - vy*atan((y-yc_left)/(x-xc_left))/pow(y-yc_left,2);

  // f << dot(xc_left) == 0;
  // f << dot(yc_left) == 0;

//   f << dot(left_bound) == 0; // Location of left wall, not to hit
//   f << dot(right_bound) == 0; // Location of right wall, not to hit
//   f << dot(upper_bound) == 0; // Corner location when unsafe
//   f << dot(max_vel) == 0;
  // f << dot(m_left) == 0;
//   f << dot(m_right) == 0;

//   f << dot(dum1) == -1*epsilon;
//   f << dot(dum2) == epsilon_vel;

  // Reference funciton
  Function h, hN;
  h << x << y << vx << vy << epsilon_x << epsilon_y << epsilon_total;
  hN << x << y;

  // Weigting Matrices
  BMatrix W = eye<bool>( h.getDim() );
  BMatrix WN = eye<bool>( hN.getDim() );

  OCP ocp(0.0,5.0,50);

  ocp.subjectTo( f );

  ocp.minimizeLSQ( W, h );

  ocp.minimizeLSQEndTerm( WN, hN );

  ocp.subjectTo( -1 <= vx <= 1 );
  ocp.subjectTo( -1 <= vy <= 1 );
  // ocp.subjectTo( -2 <= vx <= 2 );
  // ocp.subjectTo( -2 <= vy <= 2 );
//   ocp.subjectTo( pow(vx,2) + pow(vy,2) - max_vel - epsilon_vel<= 0);
//   ocp.subjectTo( left_bound - x - epsilon <= 0 );
//   ocp.subjectTo( x - epsilon - m_right*y - right_bound <= 0);
//   ocp.subjectTo( y - upper_bound - epsilon <= 0 );
  // ocp.subjectTo( m_left*(y-yc_left) - (x-xc_left) - epsilon <= 0 );
  ocp.subjectTo( x - epsilon_x <= 0 );
  ocp.subjectTo( -y - epsilon_y <= 0 );
  ocp.subjectTo( 0 <= epsilon_x);
  ocp.subjectTo( 0 <= epsilon_y );
  ocp.subjectTo( epsilon_x*epsilon_y - epsilon_total <= 0 );
  ocp.subjectTo( 0 <= epsilon_total);
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

  if (mpc.exportCode( "corner_test_export" ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	mpc.printDimensionsQP( );

	return EXIT_SUCCESS;
}