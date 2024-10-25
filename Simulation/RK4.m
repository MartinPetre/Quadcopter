function x_next = RK4(f, delta_t, x, u, drone, world)

    k1 = f(x, u, drone, world);
    k2 = f(x+delta_t*k1/2, u, drone, world);
    k3 = f(x+delta_t*k2/2, u, drone, world);
    k4 = f(x+delta_t*k3, u, drone, world);

    x_next = x + delta_t/6*(k1+2*k2+2*k3+k4);
    
end