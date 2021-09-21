%Automates optimization of a particular objective function for particular
%spring/dampoing values and swimmer physics.  Plots the resulting optimal
%gait for examination

clear all;
load('ClothoidMetricCoriolis.mat');

k = .08/10;
b = .01/10;
%.08 .01 great

[best,best2] = optimizeInertialGait_speed(funs,k,b);

[p,T] = makeGait1D6(best);
%k_best = best(7);
[displ,cost,angles,final_loop] = simulatePassiveSwimmer(p,T,funs,k,b,0);

figure(2);
clf;
plot(angles(2,:),angles(1,:));