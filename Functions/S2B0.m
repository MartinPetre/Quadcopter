function wptB0 = S2B0(wptS, invert)
    R = [1, 0      , 0       ; 
         0, cos(pi), -sin(pi); 
         0, sin(pi), cos(pi)];

    if invert
        wptB0 = pagemtimes(R',wptS);
    else
        wptB0 = pagemtimes(R,wptS);
    end

end