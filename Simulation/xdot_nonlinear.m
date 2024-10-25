function x_dot = xdot_nonlinear(state, input, drone, world)

Phi = state(1); Theta = state(2); Psi = state(3); p = state(4); q = state(5); r = state(6); u = state(7); v = state(8); w = state(9);
ft = input(1); Tx = input(2); Ty = input(3); Tz = input(4);
Ix = drone.I(1,1); Iy = drone.I(2,2); Iz = drone.I(3,3); m = drone.m;
g = world.g;

x_dot = [
    p + tan(Theta)*sin(Phi) - r*cos(Phi)*sin(Theta);
    q*cos(Phi) + r*sin(Phi);
    - q*sin(Phi)/cos(Theta) + r*cos(Phi)/cos(Theta);
    q*r*(Iy - Iz)/Ix + Tx/Ix;
    p*r*(Iz - Ix)/Iy + Ty/Iy;
    p*q*(Ix - Iy)/Iz + Tz/Iz;
    r*v - q*w - g*sin(Theta);
    p*w - r*u + g*sin(Phi)*cos(Theta);
    q*u - p*v + g*cos(Theta)*cos(Phi) - ft/m;
    u*(cos(Theta)*cos(Psi)) + v*(cos(Psi)*sin(Phi)*sin(Theta) - cos(Phi)*sin(Psi)) + w*(cos(Phi)*cos(Psi)*sin(Theta) + sin(Phi)*sin(Psi));
    u*(cos(Theta)*sin(Psi)) + v*(sin(Phi)*sin(Psi)*sin(Theta) + cos(Phi)*cos(Psi)) + w*(cos(Phi)*sin(Psi)*sin(Theta) - cos(Psi)*sin(Phi));
    -u*(sin(Theta)) + v*(cos(Theta)*sin(Phi)) + w*(cos(Phi)*cos(Theta))
    ];

end