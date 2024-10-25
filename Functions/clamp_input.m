function u_sat = clamp_input(u, drone)
    
    omega_squared = drone.act_mat\u;
    
    omega_squared = min(max(omega_squared, 0), (drone.max_rpm*2*pi/60)^2);
    
    u_sat = drone.act_mat*omega_squared;

end