init_drone

f = @xdot_nonlinear; %Nonlinear continuous dynamics function handle
xBar = drone.xg; %State operating point
uBar = drone.ug; %Input operating point

%% LQR controller
% controller_lqr = [uBar + L*xBar, -L]; % [ff-term, fb term]

controller_lqr = [uBar, -L]; % [ff-term, fb term]

%% Trajectory Generation
satisfied = false;
while ~satisfied
    course_choice = input('Choose course: Hover - (1), Straight - (2), Box - (3), Ellipse - (4), To Goal - (5) \n');
    
    t_end = input('Provide duration of simulation [sec]: ');
    
    [TrajectoryB0, TrajectoryS, wpts] = nominal_trajectory(course_choice, t_end, drone);
    
    if exist(append(pwd,'\missionwaypoints.mat'), 'file') == 2
        delete(append(pwd,'\missionwaypoints.mat'));
    end

    figure()
    subplot(2,1,1)
    plot(TrajectoryB0.Time.vec', TrajectoryB0.Trajectory(1:3,:))
    xlabel('time [s]')
    ylabel('[m]')
    legend('X', 'Y', 'Z')
    grid on
    
    subplot(2,1,2)
    plot(TrajectoryB0.Time.vec', TrajectoryB0.Trajectory(4,:)')
    xlabel('time [s]')
    ylabel('[rad]')
    grid on
    
    satisfiedQA = input('Satisfied with the trajectory? (1=YES): ','s');
    
    if strcmp(satisfiedQA, '1')
        satisfied = true;
    end
end

%% Simulation
delta_t = 0.01;
[X, U, T, R_B2B0] = simulate(drone, world, controller_lqr, TrajectoryB0, f, delta_t, t_end);

%% Visualization
animate_flight(drone, X, U, T, R_B2B0)

% %% Simulation with Simulink
% Xref = [T', repmat(xBar(10), length(T), 1)];
% Yref = [T', repmat(xBar(11), length(T), 1)];
% Zref = [T', repmat(xBar(12), length(T), 1)];
% YAWref = [T', repmat(xBar(3), length(T), 1)];
% out = sim("SimulationEnv.slx", t_end);
% 
% x_simulink = out.state.Data;
% 
%% Plotting
labelsx = {'\Phi', '\Theta', '\Psi', 'p', 'q', 'r', 'u', 'v', 'w', 'x', 'y', 'z'};
figure
for i = 1:12
    subplot(4,3,i)
    plot(T, X(i,:))

    ylabel(labelsx{i})
    xlabel('Time [s]')
end

u_max = zeros(4,1);
u_min = zeros(4,1);
f_max = drone.act_mat*ones(4,1)*(10000*2*pi/60)^2;
u_max(1) = f_max(1);
u_min(1) = 0;
tau_xy_max = drone.act_mat*[0; 0; (drone.max_rpm*2*pi/60)^2; (drone.max_rpm*2*pi/60)^2];
u_max(2:3) = tau_xy_max(2);
u_min(2:3) = -tau_xy_max(2);
tau_z_max = drone.act_mat*[0; (drone.max_rpm*2*pi/60)^2; 0; (drone.max_rpm*2*pi/60)^2];
u_max(4) = tau_z_max(4);
u_min(4) = -tau_z_max(4);

labelsu = {'f_t', '\tau_x', '\tau_y', '\tau_z'};
figure
for i = 1:4
    subplot(1,4,i)
    hold on
    stairs(T(1:end-1), U(i,:))
    yline(u_max(i), 'r-', 'max input', 'LineWidth', 2)
    yline(u_min(i), 'r-', 'min_input', 'LineWidth', 2)
    hold off

    ylabel(labelsu{i})
    xlabel('Time [s]')
end