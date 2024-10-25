% Author: Ludwig Horvath, Martin Petré

% Date 12/19/2023

classdef DRONE

    properties
        % Physical parameters

        % Inertia
        m (1,1) double {mustBeReal, mustBeFinite}
        I (3,3) double {mustBeReal, mustBeFinite}
        
        % Distance to Actuator
        r (1,1) double {mustBeReal, mustBeFinite}

        % Propellers
        b (1,1) double {mustBeReal, mustBeFinite}       % Lift Coefficient
        d (1,1) double {mustBeReal, mustBeFinite}       % Drag Coefficient
        max_rpm (1,1) double {mustBeReal, mustBeFinite} % Maximum rpm

        % Electronics
        k_t (1,1) double {mustBeReal, mustBeFinite}   % Motor Kt-Value
        k_v (1,1) double {mustBeReal, mustBeFinite}   % Motor Kv-Value
        Q (1,1) double {mustBeReal, mustBeFinite}     % Battery mAh
        U (1,1) double {mustBeReal, mustBeFinite}     % Battery nominal Voltage
        % State/input size
        state_size (1,1) int16 {mustBeReal, mustBeFinite}
        input_size (1,1) int16 {mustBeReal, mustBeFinite}
        
        % Initial state and goal operating point
        x0 (12,1) double {mustBeReal, mustBeFinite}
        xg (12,1) double {mustBeReal, mustBeFinite}
        ug (4,1) double {mustBeReal, mustBeFinite}
        
        % Microcontroller sampling time
        h (1,1) double {mustBeReal, mustBeFinite}

        % Actuation matrix
        act_mat (4,4) double {mustBeReal, mustBeFinite}
    end
    
    methods
        function obj = DRONE(m, I, r, b, d, k_v, Q, U, h, g)
            obj.m = m;
            obj.I = I;
            obj.r = r;
            obj.b = b;
            obj.d = d;
            obj.k_v = k_v;
            obj.k_t = 1/k_v;
            obj.Q = Q;
            obj.U = U;

            % Initial state, in S coordinates
            obj.x0 = [0;0;0;0;0;0;0;0;0;0;0;0];

            % Goal operating point, in S coordinates
            obj.xg = [0;0;0;0;0;0;0;0;0;5;0;5];

            % Goal operating input (hovering)
            obj.ug = [m*g;0;0;0];

            obj.state_size = size(obj.x0,1);
            obj.input_size = size(obj.ug,1);

            obj.h = h;
            
            obj.act_mat = [b          ,  b          ,   b          ,   b          ; ...
                          -b*r/sqrt(2), -b*r/sqrt(2), b*r/sqrt(2)  ,   b*r/sqrt(2); ...
                           b*r/sqrt(2),  -b*r/sqrt(2), -b*r/sqrt(2), b*r/sqrt(2)  ; ...
                          -d          ,            d,           - d,             d];

            obj.max_rpm = 10000;
        end
    end
end



