classdef jackal < UGV
    methods
        function obj = jackal(map)
            obj = obj@UGV(map);
            obj.physical.W = 0.4;
            obj.physical.L = 0.5;
            obj.physical.wheel_radius = 0.075;
            obj.physical.v_max = 2.0;
            obj.physical.omega_max = 1.0;
            obj.dd = DifferentialDrive(obj.physical.L,obj.physical.wheel_radius);
        end
    end
end