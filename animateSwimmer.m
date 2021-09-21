%Generates animated gif of swimming movements/displacements for the three
%link swimmer

function animateSwimmer(sol,sys)

    figure(10);
    t = [0:1/30:sys.animate_T];
    
    for i = 1:numel(sol(1,:))
       
        clf;
        axis equal;
        hold on;
        
        X = sol(1,i);
        Y = sol(2,i);
        Theta = sol(3,i);
        
        a1 = sol(4,i);
        a2 = sys.p.rc(t(i));
        
        thetas = linspace(0,2*pi,100);
        ellipseX = .5/3*cos(thetas);
        ellipseY = .05/3*sin(thetas);
        
        [centerX,centerY] = shiftPoints(ellipseX,ellipseY,Theta,X,Y);
        
        [leftX,leftY] = shiftPoints(ellipseX,ellipseY,-a1,-.5/3*cos(a1),.5/3*sin(a1));
        [leftX,leftY] = shiftPoints(leftX,leftY,0,-.5/3,0);
        [leftX,leftY] = shiftPoints(leftX,leftY,Theta,X,Y);
        
        [rightX,rightY] = shiftPoints(ellipseX,ellipseY,a2,.5/3*cos(a2),.5/3*sin(a2));
        [rightX,rightY] = shiftPoints(rightX,rightY,0,.5/3,0);
        [rightX,rightY] = shiftPoints(rightX,rightY,Theta,X,Y);
        
        motorX = .05/2*cos(thetas)+.5/3;
        motorY = .05/2*sin(thetas);
        [motorX,motorY] = shiftPoints(motorX,motorY,Theta,X,Y);
        
        bl = [0    0.4471    0.7412];
        fill(centerX,centerY,bl);
        fill(leftX,leftY,bl);
        fill(rightX,rightY,bl);
        fill(motorX,motorY,'r');
        
        axis([-3,3,-3,3]);
        drawnow;
        
        if i == 1
            gif('FreeSwimAnimation.gif','DelayTime',1/15);
        else
            gif;
        end
        
    end

end

function R = getRotMatrix(theta)

    R = [cos(theta),-sin(theta);sin(theta),cos(theta)];

end

%Rotates and translates datapoints
function [newX,newY] = shiftPoints(x,y,theta,dx,dy)

    R = getRotMatrix(theta);
    
    allPoints = [x;y];
    rotatedPoints = R*allPoints;
    
    newX = rotatedPoints(1,:) + dx;
    newY = rotatedPoints(2,:) + dy;

end