% simulate: function to simulate nonlinear plant
%
% Inputs:
%       drone:                  Object containing drone parameters, state/
%                               input size
%       world:                  Object containing world constants and
%                               parameters
%       controller:             Choosen controller
%       f:                      Continuous nonlinear dynamics function
%                               handle
%       delta_t:                Time step used in simulation
%       t_end:                  Simulation end time
%
% Outputs:
%       X:                      State evolution in a matrix of size [9xN+1]
%       U:                      Input evolution in a matrix of size [4xN]
%       T:                      Corresponding time vector [1xN+1]
%                               where N stands for the number of datapoints
%
% Author: Martin Petré mapetr@kth.se

function [X, U, T, R_B2B0] = simulate(drone, world, controller, TrajectoryB0, f, delta_t, t_end)

    k = 0;
    l = 1;
    
    nr_of_datapoints = floor(t_end/delta_t)+1;
    h = drone.h; % Sampling time
    
    X = zeros(drone.state_size, nr_of_datapoints+1);
    R_B2B0 = zeros(3, 3, nr_of_datapoints+1);
    U = zeros(drone.input_size, nr_of_datapoints);

    xref = TrajectoryB0.Trajectory;

    x = drone.x0;
    x(10:12) = S2B0(x(10:12), false);
    X(:,1) = x;
    i = zeros(4,1);

    R_B2B0(:,:,l) = B02B(x(1), x(2), x(3))';
    
    for t=0:delta_t:t_end

        if t >= k*h %ZOH sampling
            
            xBar = [0;0;xref(4,k+1);0;0;0;0;0;0;xref(1:3,k+1)];
            
            switch controller.type
                case 'LQR'
                    u = controller.gain*[1; (x-xBar)];

                case 'LQRI'
                    output_index = [3, 10, 11, 12];
                    r = xBar(output_index);
                    y = x(output_index);
                    
                    x_augmented = [x; i];
                    u = controller.gain*[1; x_augmented];

                    i = i + (r-y);
            end

            u = clamp_input(u, drone); % Saturate input

            k = k + 1;
        end
        x_next = RK4(f, delta_t, x, u, drone, world);

        U(:,l) = u;
        X(:,l+1) = x_next;
        x = x_next;
        R_B2B0(:,:,l+1) = B02B(x(1), x(2), x(3))'; % Notice the transpose to get B2B0

        l = l+1;
    end

    T = 0:delta_t:nr_of_datapoints*delta_t;
end