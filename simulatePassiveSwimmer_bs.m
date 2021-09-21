%Simulates the inertial-passive gait for the breaststroke swimming system
%with the aim of finding the limit cycle

%Identical to simulatePassiveSwimmer.m but with an asymmetric spring

%p-position/velocity/acceleration of active joint as function wrt time
%T-period of gait
%funs-metric/coriolis/connection matrices as functions wrt shape
%k1-spring constant on one side of joint from 0 deflection
%k2-spring constant on other side of joint from k1
%b-damping constant
%animate-boolean, whether or not to make a gif of the resulting motion

%displ-displacement due to limit cycle gait
%cost-torque squared cost to execute limit cycle gait
%angles-joint deflection profile of limit cycle gait
%final_loop-[X motion;Y motion;Theta motion;passive joint;controlled
%joint;cost] over course of limit cycle gait

function [displ,cost,angles,final_loop] = simulatePassiveSwimmer_bs(p,T,funs,k1,k2,b,animate)

%Store inputs into system structure
sys.k1 = k1;
sys.k2 = k2;
sys.d = b;
sys.A = funs.A_fun;
sys.dMdr1 = funs.dmdr1_fun;
sys.dMdr2 = funs.dmdr2_fun;
sys.metric = funs.metric_fun;
sys.p = p;
sys.T = T;
sys.animate_T = 5;

%Initial values for gait
init = [0;0;0;0;0;0];

%Number of cycles to run before evaluating limit cycle
t_lim = 3*T;

%Set function to evaluate gait dynamics ODE
odefun = @(t,X) getDispAndCost(t,X,sys);
%Evaluate ODE
sol = ode45(odefun,[0,t_lim],init);

%Make even timesteps across limit cycle
nsteps = 100;
dt = T/nsteps;
t = [0:dt:T];

%Initial values of limit cycle loop are ending values of initial loops with
%zeroed displacements and cost
final_loop_start = sol.y(:,end);
final_loop_start(1:3) = [0;0;0];
final_loop_start(end+1) = 0;

%Evaluate ODE for limit cycle
costDispCalc = @(t,X) getDispAndCost(t,X,sys);
sol_final = ode45(costDispCalc,[0,T],final_loop_start);
sol_final = deval(sol_final,t);

%ODE outputs for outside analysis
final_loop = sol_final;

%Store gait shape positions and velocities
angles = zeros(4,numel(final_loop(1,:)));
angles(1,:) = final_loop(4,:);
angles(2,:) = sys.p.rc(t);
angles(3,:) = final_loop(5,:);
angles(4,:) = sys.p.drc(t);

%Calculate total displacements and cost
gx = final_loop(1,end);
gy = final_loop(2,end);
displ = sqrt(gx^2 + gy^2);
cost = final_loop(6,end);

%If making a gif of output motion, do it
if animate
    t_span = [0,sys.animate_T];
    sol = ode45(odefun,[0,t_span(2)],init);
    t = [0:1/30:t_span(2)];
    sol_a = deval(sol,t);  
    animateSwimmer(sol_a,sys);
end

end

%ODE descriptor for swimmer dynamics with cost included
function dF = getDispAndCost(t,X,sys)

    dF = zeros(6,1);
    
    %Get current position/velocity of passive joint
    r1 = X(4);
    dr1 = X(5);
    %Get current position/velocity/acceleration of control joint
    r2 = sys.p.rc(t);
    dr2 = sys.p.drc(t);
    ddr2 = sys.p.ddrc(t);
    
    %Get spring constant for asymmetric spring
    if r1 <= 0
        k = sys.k1;
    else
        k = sys.k2;
    end
    
    %Get torque on passive joint from passive components
    tau_1 = -sys.k*r1 - sys.d*dr1;

    %Wrap joints to pi so they stay in bounds of interpolant functions
    r1 = wrapToPi(r1);
    r2 = wrapToPi(r2);
    
    %Interpolate metric from shape
    M = sys.metric(r1,r2);
    %Get coriolis forces from shape and shape velocity
    C = getCoriolisForces(r1,dr1,r2,dr2,sys);
    
    %Calculate acceleration of passive joint using dynamic equation
    ddr1 = (tau_1 - M(1,2)*ddr2 - C(1))/M(1,1);
    %Get required torque at control joint to execute
    tau_2 = M(2,:)*[ddr1;ddr2] + C(2);
    
    %Interpolate connection
    A = sys.A(r1,r2);
    %Reconstruction equation to get swimmer motion
    g_circ = -A*[dr1;dr2];
    
    %Convert swimmer velocity from local to world coordinates
    theta = X(3);
    g = [cos(theta),-sin(theta),0;...
        sin(theta),cos(theta),0;...
        0,0,1];
    g_dot = g*g_circ;
    
    %Store rate of change of each parameter in ODE
    %World position velocities
    dF(1:3) = g_dot;
    %Passive joint velocity/acceleration
    dF(4:5) = [dr1;ddr1];
    %Cost velocity
    dF(6) = tau_2^2;
    
end

%Calculates coriolis forces from system shape/velocity and derivatives of
%metric
function C = getCoriolisForces(r1,dr1,r2,dr2,sys)

    dr = [dr1;dr2];
    
    C1 = sys.dMdr1(r1,r2)*dr1*dr + sys.dMdr2(r1,r2)*dr2*dr;
    C2 = -.5*[dr'*sys.dMdr1(r1,r2)*dr;dr'*sys.dMdr2(r1,r2)*dr];
    
    C = C1 + C2;
end
