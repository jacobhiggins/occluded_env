% State filter for the AMR class
classdef StateFilter < handle
    properties
        models = struct("continuous",ss(),"discrete",ss());
        process;
        measure;
        measured_state;
        est_state;
        est_cov;
        prev_filter_time = 0.0;
        filter_dt;
    end
    methods
        function obj = StateFilter(measure, process, dt, init_est_state)
            obj.filter_dt = dt;
            obj.models.continuous  = process.model_continuous;
            obj.models.discrete = c2d(process.model_continuous,dt);
            obj.process = process;
            obj.measure = measure;
            obj.est_state = init_est_state;
            obj.est_cov = process.noise.var;
        end
        function measured_state = measure_state(obj,robot)
           C = obj.models.continuous.C;
           x = robot.process.state;
           measured_state = C*x + mvnrnd(zeros(size(obj.models.discrete.C,1),1),obj.measure.noise.var)';
        end
        function est_state = estimate(obj,robot)
            if (robot.time - obj.prev_filter_time) < 0.99*obj.filter_dt
                est_state = obj.est_state;
               return 
            end
            obj.prev_filter_time = robot.time;
            
            obj.measured_state = obj.measure_state(robot);
            input = robot.process.input;
            A = obj.models.discrete.A;
            B = obj.models.discrete.B;
            C = obj.models.discrete.C;
            D = obj.models.discrete.D;
            state_predict = A*obj.est_state + B*input;
            measure_predict = C*state_predict;
            cov_predict = A*obj.est_cov*A' + obj.process.noise.var;
            measure_residual = obj.measured_state - measure_predict;
            kalman_gain = cov_predict*C'/(C*cov_predict*C' + obj.measure.noise.var);
            obj.est_state = state_predict + kalman_gain*measure_residual;
            obj.est_cov = (eye(size(kalman_gain*C)) - kalman_gain*C)*cov_predict;
            
            est_state = obj.est_state;
        end
    end
end