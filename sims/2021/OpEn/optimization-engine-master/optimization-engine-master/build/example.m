nu = 6; % number of decision variables
np = 2; % number of parameters

u = casadi.SX.sym('u',nu); % decision variables
p = casadi.SX.sym('p',np); % parameters

phi = rosenbrock(u, p); % cost function

constraints = OpEnConstraints.make_ball_at_origin(1.0); 

builder = OpEnOptimizerBuilder().with_problem(u, p, phi, constraints);
optimizer = builder.build();

optimizer.run();
optimizer.connect();

p = [1, 250]; % parameter
out = optimizer.consume(p);

optimizer.disconnect();
optimizer.stop();