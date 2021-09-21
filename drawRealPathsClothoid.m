%Draws optimal gait for the clothoid swimmer on the clothoid CCF

clear all;
warning('off','all');
addpath('DataFiles');

%Import gait, CCF data, and clothoid physics
load('ClothoidSpeedGait.mat');
load('ClothoidCCFData.mat');
load('ClothoidMetricCoriolis.mat');

rossred = [234 14 30]/255;

%Import best gait fourier params
xs = best;

%Generate motion based off number of fourier params
if numel(xs) < 10
    [p,T] = makeGait1D6(xs);
else
    [p,T] = makeGait1D(xs);
end

%If we also optimized for the spring constant
if numel(xs) == 7
    %Load spring constant
    k = xs(end)/1000;
else
    %Default spring constant
    k = .08/10;
end

%Set damping parameter
b = .01/10;

%Run simulation for optimal gait
[displ,cost,angles,final_loop] = simulatePassiveSwimmer(p,T,funs,k,b,0);

%Calculate and display useful info for optimal gait
disp(['Forward Speed: ',num2str(displ/T)]);
disp(['Period: ',num2str(T)]);
disp(['Max Motor Speed: ',num2str(max(abs(angles(4,:))))]);
costs = final_loop(6,:);
dt = T/100;
%final cost = sum(sqrt(dcost*dt)) -> Undo this
dcosts = diff(costs);
dcosts = sqrt(dcosts);
dcosts = dcosts/dt;
maxTorque = max(abs(dcosts));
disp(['Max Motor Torque: ',num2str(maxTorque)]);

%Get total X,Y,Theta displacement from gait
gx = final_loop(1,end);
gy = final_loop(2,end);
gtheta = final_loop(3,end);

%Plot CCF contours
Hx = H{4};
Hx = Hx';
a_c = grid{2}';
a_p = grid{1}';
figure(13);
clf;
contour(a_c,a_p,Hx,7,'linewidth',2);
axis equal;
axis tight;
colormap(Colorset.colormap_contour); 
hold on;

%Clothoid gait sometimes is vertically offset, fix this
angles(1,:) = angles(1,:) - sum(angles(1,:))/numel(angles(1,:));
%Change starting phase of gait for arrow generation
phaseShiftPercent = .04;
phaseGrab = floor(size(angles,2)*phaseShiftPercent)+1;
angles = [angles(:,phaseGrab:end),angles(:,1:phaseGrab)];

%Plot gait
lw = 3;
plot(angles(2,:),angles(1,:),'Color',rossred,'LineWidth',lw);

%Generate and plot arrowheads for gait directions
heads = plot_dir_arrows(angles(2,:),angles(1,:),2);
for i = 1:numel(heads)
    plot(heads(i).XData,heads(i).YData,'Color',rossred,'LineWidth',lw);
end

%Set plot ticks
xticks([-pi/2,0,pi/2]);
yticks([-2.5,0,2.5]);
xticklabels('');
yticklabels('');

warning('on','all');


        