function [Q,R] = generate_QR_brysons_rule(x_max, drone)

    Q = diag(1./x_max.^2);

    max_f_error = drone.ug(1)*0.01;

    max_tau_xy_error = 0.01;
    
    max_tau_z_error = 0.01;
    
    R = diag([1/max_f_error^2, 1/max_tau_xy_error^2, 1/max_tau_xy_error^2, 1/max_tau_z_error^2]);

end
