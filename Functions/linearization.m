function [A, B, A_d, B_d] = linearization(drone, world)
    syms Phi Theta Psi p q r u v w x y z Ix Iy Iz Tx Ty Tz g ft m h
    
    xBar = drone.xg; uBar = drone.ug; g_num = world.g; I = drone.I; m_num = drone.m; h_num = drone.h;
    
    X = [Phi; Theta; Psi; p; q; r; u; v; w; x; y; z];
    input = [ft; Tx; Ty; Tz];
    
    f = [
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
    
    for j = 1:12
        for i = 1:12
            A_diff(j,i) = diff(f(j), X(i));
        end
    end
    
    for j = 1:12
        for i = 1:4
            B_diff(j,i) = diff(f(j), input(i));
        end
    end     
    
    A = subs(A_diff, X, xBar);
    B = subs(B_diff, input, uBar);
    
    A_d = expm(A*h);
    B_d = int(A_d,h)*B;

    A_d = double(subs(A_d, {'g', 'h'}, [g_num, h_num]));
    B_d = double(subs(B_d, {'g', 'h', 'Ix', 'Iy', 'Iz', 'm'}, [g_num, h_num, I(1,1), I(2,2), I(3,3), m_num]));

    A = double(subs(A, {'g'}, g_num));
    B = double(subs(B, {'Ix', 'Iy', 'Iz', 'm'}, [I(1,1), I(2,2), I(3,3), m_num]));

end
