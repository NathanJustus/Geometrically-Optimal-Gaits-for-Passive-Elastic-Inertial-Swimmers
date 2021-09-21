%Optimizes inertial-passive gait for desired objective function

function bestXs = optimizeInertialGait_speed(funs,k,b)

%Boolean whether or not to optimize spring constant too
optimK = 0;

%Initialize fourier parameters
y0 = zeros(1,10);
y0(3) = 1.5;
y0(10) = 20;

%Global holder for last loop's resulting gait for constraint checking
global lastAngs;

%Set optimization options
options = optimoptions('fmincon','Display','iter','MaxFunctionEvaluations',1000,'Algorithm','sqp','DiffMinChange',0.1);

%Set lower and upper bounds for fourier parameters
lim = 2;
ub = zeros(1,numel(y0));
ub(3) = 1.5;
ub(end) = 20;
lb = -1*ub;
lb(end) = pi;

%If optimizing the spring constant too, add to list of optimization params
if optimK
    springInd = numel(y0) + 1;
    y0(springInd) = 1000*k;
    ub(springInd) = 10;
    lb(springInd) = 0;
end

%Set optimization objective function
fun = @(x) getDispCost(x,funs,k,b,optimK);
%Set optimization constraint function
nonlcon = @(x) nonlinearConstraints(x,funs);

%Run optimization and output best parameters
bestXs = fmincon(fun,y0,[],[],[],[],lb,ub,nonlcon,options);

end

%Objective function for optimization
function cost = getDispCost(xs,funs,k,b,optimK)

    %Storage for gait results for constraint checking in different function
    global lastAngs;
    
    %If optimizing the spring constant, load it
    if optimK
        k = xs(end)/1000;
    end
    
    %Show what values we are currently checking
    disp(xs)
    
    %Load gait based off fourier parameters
    if numel(xs) < 10
        [p,T] = makeGait1D6(xs);
    else
        [p,T] = makeGait1D(xs);
    end

    %Simulate a few runs of the current gait parameters starting from rest
    [displ,cost,angles,final_loop] = simulatePassiveSwimmer(p,T,funs,k,b,0);
    
    %Calculate distance between start of ending loop and end of ending loop
    da1 = angles(1,1)-angles(1,end);
    da2 = angles(2,1)-angles(2,end);
    err = sqrt(da1^2+da2^2);
    
    %Calculate speed of gait
    speed = abs(displ/T);
    
    %Display gait statistics during optimization
    disp(['Error: ',num2str(err),', Speed: ',num2str(speed)]);
    
    lastAngs = angles;

    %Check constraint violations
    C = nonlinearConstraints(xs,funs);

    %Display if constraints were violated
    if sum(C<0) ~= 2
        disp('Constraints Violated');
    end
    
    %Scaling term for metabolic cost
    metabolic = 0.01;
    
    %Speed objective function
    cost = -speed;
%     %Energetic efficiency objective function
%     cost = -abs(displ/cost);
%     %Metabolic objective function
%     cost = -abs(displ/(metabolic*T+cost));
    
end 

%Nonlinear constraints
%Makes sure that controlled and passive joints stay in desired bounds
%for a given gait description from fourier parameters
function [C,Ceq] = nonlinearConstraints(x,funs)

    global lastAngs;

    %Make gait from fourier parameters
    if numel(xs) < 10
        [p,T] = makeGait1D6(xs);
    else
        [p,T] = makeGait1D(xs);
    end
    
    %Load gait results from last simulation
    angles = lastAngs;
    %Keep only position data and drop velocity data
    angles = angles(1:2,:);
    
    %Set absolute bounds for controlled and passive joints
    constrainJoint_c = 2*pi/3;
    constrainJoint_p = 2*pi;
    
    %Initial constraint matrix
    constraint = [constrainJoint_p;constrainJoint_c];
    
    %Max absolute values of active and passive joint positions
    max_rs = max(abs(angles),[],2);
    %Amount that they overshoot constraints
    max_rs = max_rs - constraint;
    
    %Save for fmincon to keep constraints
    Ceq = [];
    C = [max_rs];

end