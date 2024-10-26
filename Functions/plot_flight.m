function plot_flight(drone, X, U, T, TrajectoryB0)

% %% Simulation with Simulink
% Xref = [T', repmat(xBar(10), length(T), 1)];
% Yref = [T', repmat(xBar(11), length(T), 1)];
% Zref = [T', repmat(xBar(12), length(T), 1)];
% YAWref = [T', repmat(xBar(3), length(T), 1)];
% out = sim("SimulationEnv.slx", t_end);
% 
% x_simulink = out.state.Data;

labelsx = {'\Phi', '\Theta', '\Psi', 'p', 'q', 'r', 'u', 'v', 'w', 'x', 'y', 'z'};
figure('Position', [1920/2, 500, 1920/2, 450]); 
for i = 1:12
    subplot(4,3,i)
    plot(T, X(i,:))

    if i==3 || i>9
        hold on
        k = mod(i+1,10);
        plot(TrajectoryB0.Time.vec, TrajectoryB0.Trajectory(k,:))
        legend('true', 'reference')
    end

    ylabel(labelsx{i})
    xlabel('Time [s]')
end

u_max = zeros(4,1);
u_min = zeros(4,1);
f_max = drone.act_mat*ones(4,1)*(drone.max_rpm*2*pi/60)^2;
u_max(1) = f_max(1);
u_min(1) = 0;
tau_xy_max = drone.act_mat*[0; 0; (drone.max_rpm*2*pi/60)^2; (drone.max_rpm*2*pi/60)^2];
u_max(2:3) = tau_xy_max(2);
u_min(2:3) = -tau_xy_max(2);
tau_z_max = drone.act_mat*[0; (drone.max_rpm*2*pi/60)^2; 0; (drone.max_rpm*2*pi/60)^2];
u_max(4) = tau_z_max(4);
u_min(4) = -tau_z_max(4);

labelsu = {'f_t', '\tau_x', '\tau_y', '\tau_z'};

figure('Position', [1920/2, 0, 1920/2, 410]);  
for i = 1:4
    subplot(1,4,i)
    hold on
    stairs(T(1:end-1), U(i,:))
    yline(u_max(i), 'r-', 'max input', 'LineWidth', 2)
    yline(u_min(i), 'r-', 'min input', 'LineWidth', 2)
    hold off

    ylabel(labelsu{i})
    xlabel('Time [s]')
end

end