init_drone

f = @xdot_nonlinear; %Nonlinear continuous dynamics function handle
xBar = drone.xg; %State goal/operating point
uBar = drone.ug; %Input goal/operating point

%% LQR controller
% controller_lqr = [ug + L*xg, -L]; % [ff-term, fb term]
controller_lqr = LQCONTROLLER('LQR', [uBar, -L]);

%% LQR + integrator
controller_lqri = LQCONTROLLER('LQRI', [uBar, -LI]);

%% PID

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
controller = controller_lqri; % Choose controller
delta_t = 0.01;
[X, U, T, R_B2B0] = simulate(drone, world, controller, TrajectoryB0, f, delta_t, t_end);

%% Visualization
animate_flight(drone, X, U, T, R_B2B0)

%% Plotting
plot_flight(drone, X, U, T, TrajectoryB0)