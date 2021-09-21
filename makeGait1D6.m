%Generates active joint position/velocity/acceleration as functions of time
%using fourier fit data for a second order fit

%y = [a0,a1,b1,a2,b2,w]

function [gait,T] = makeGait1D6(y) 
        
    w = abs(y(6));
    gait.rc = @(t) y(1)+y(2)*cos(w*t)+y(3)*sin(w*t)+y(4)*cos(2*w*t)+...
                            +y(5)*sin(2*w*t);
    gait.drc = @(t) -w*y(2)*sin(w*t)+w*y(3)*cos(w*t)-2*w*y(4)*sin(2*w*t)+...
                              +2*w*y(5)*cos(2*w*t);
    gait.ddrc = @(t) -w^2*y(2)*cos(w*t)-w^2*y(3)*sin(w*t)-4*w^2*y(4)*cos(2*w*t)+...
                               -4*w^2*y(5)*sin(2*w*t);
    T = 2*pi/w;
                
end