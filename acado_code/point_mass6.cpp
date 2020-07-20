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
  DifferentialState xc;
  DifferentialState yc;
  DifferentialState ax;
  DifferentialState ay;
  DifferentialState phi;
  DifferentialState safe;
  // DifferentialState xc;
  // DifferentialState yc;
  DifferentialState left_bound;
  DifferentialState right_bound;
  DifferentialState delta_x;
  DifferentialState m;  // Slope for robot-to-corner boundary

  Control ax_dot;
  Control ay_dot;
  Control epsilon;
  Control epsilon2; // For safety bound

  DifferentialEquation f;


  f << dot(x) == vx;
  f << dot(y) == vy;
  f << dot(vx) == ax;
  f << dot(vy) == ay;
  f << dot(xc) == epsilon;
  f << dot(yc) == epsilon2;
  f << dot(ax) == ax_dot;
  f << dot(ay) == ay_dot;
  // f << dot(m) == vx/(yc - y) - vy*(xc - x)/pow(yc - y,2);

  // f << dot(phi) == ( vx/(yc - y) - vy*(xc - x)/pow(yc - y,2) )/( 1 + pow((y - yc)/(x - xc),2) ); // This one is the best so far
  // f << dot(phi) == ( vx/(yc - y) - vy*(xc - x)/pow(yc - y,2) ); // Negtaive inverse slope
  // f << 0 == end - xc;

  // f << dot(phi) == ( -vy/(xc - x) + vx*(yc - y)/pow(xc - x,2) )/( 1 + pow((y - yc)/(x - xc),2) ); // Correct theta
  // f << dot(phi) == (vy/x - ((y*vx)/pow(x,2)))/sqrt(pow(y,2)+pow(x,2))/(1 + pow(y/x,2)) - atan(y/x)*(x*vx + y*vy)/pow(pow(x,2)+pow(y,2),3/2); // theta / distance
  f << dot(phi) == (vy/x - ((y*vx)/pow(x,2)))/(1 + pow(y/x,2))/y - atan(y/x)*vy/pow(y,2); // Phi over y

  f << dot(safe) == (ax*vx + ay*vy)/(delta_x); // Safety objective
  // f << dot(safe) == (ax*vx + ay*vy);

  // f << dot(phi) == ( -vy/(xc - x) + vx*(yc - y)/pow(xc - x,2) )/( 1 + pow((y - yc)/(x - xc),2) )/pow( pow(y-yc,2) + pow(x-xc,2) + 1, 0.5 );
  // f << 0 == d_0 - sqrt(pow(yc - y,2) + pow(xc - x,2)) - 8;
  // f << dot(xc) == 0;
  // f << dot(yc) == 0;
  // f <<  0 == slope - (yc - y)/(xc - x);
  // f << 0 == angle - atan((yc - y)/(xc - ));

  f << dot(left_bound) == 0; // Location of left wall, not to hit
  f << dot(right_bound) == 0; // Location of right wall, not to hit
  f << dot(delta_x) == 0; // safe distance provided by program
  f << dot(m) == 0;

  // Reference funciton
  Function h, hN;
  h << x << y << phi << safe << ax_dot << ay_dot << epsilon << epsilon2;
  hN << x << y << phi << safe;

  // Weigting Matrices
  BMatrix W = eye<bool>( h.getDim() );
  BMatrix WN = eye<bool>( hN.getDim() );
  // W(0,0) = 1;
  // WN(0,0) = 1;
  // W(2,2) = 100; // for good run, both weihts = 500
  // WN(2,2) = 100;

  // WN *= 5;

  OCP ocp(0.0,5.0,50);

  ocp.subjectTo( f );

  ocp.minimizeLSQ( W, h );

  ocp.minimizeLSQEndTerm( WN, hN );

  ocp.subjectTo( -5 <= ax <= 5 );
  ocp.subjectTo( -5 <= ay <= 5 );
  ocp.subjectTo( -20 <= vx <= 20 );
  ocp.subjectTo( -20 <= vy <= 20 );
  ocp.subjectTo( safe - 1 - epsilon2 <= 0 );
  ocp.subjectTo( left_bound - x - epsilon <= 0 );
  ocp.subjectTo( x - epsilon - m*y - right_bound <= 0);
  ocp.subjectTo( 0 <= epsilon);
  ocp.subjectTo( 0 <= epsilon2);
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

  if (mpc.exportCode( "point_mass_export6" ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	mpc.printDimensionsQP( );

	return EXIT_SUCCESS;
}
