%Generates active joint position/velocity/acceleration as functions of time
%using fourier fit data 

%y = [a0,a1,b1,a2,b2,a3,b3,a4,b4,w]

function [gait,T] = makeGait1D(y) 
        
    w = abs(y(10));
    gait.rc = @(t) y(1)+y(2)*cos(w*t)+y(3)*sin(w*t)+y(4)*cos(2*w*t)+...
                            +y(5)*sin(2*w*t)+y(6)*cos(3*w*t)+y(7)*sin(3*w*t)+...
                            +y(8)*cos(4*w*t)+y(9)*sin(4*w*t);
    gait.drc = @(t) -w*y(2)*sin(w*t)+w*y(3)*cos(w*t)-2*w*y(4)*sin(2*w*t)+...
                              +2*w*y(5)*cos(2*w*t)-3*w*y(6)*sin(3*w*t)+3*w*y(7)*cos(3*w*t)+...
                              -4*w*y(8)*sin(4*w*t)+4*w*y(9)*cos(4*w*t);
    gait.ddrc = @(t) -w^2*y(2)*cos(w*t)-w^2*y(3)*sin(w*t)-4*w^2*y(4)*cos(2*w*t)+...
                               -4*w^2*y(5)*sin(2*w*t)-9*w^2*y(6)*cos(3*w*t)-9*w^2*y(7)*sin(3*w*t)+...
                               -16*w^2*y(8)*cos(4*w*t)-16*w^2*y(9)*sin(4*w*t);
    T = 2*pi/w;
                
end