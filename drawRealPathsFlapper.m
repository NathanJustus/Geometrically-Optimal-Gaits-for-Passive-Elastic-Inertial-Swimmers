%Draws optimal gait for the breaststroke swimmer on the CCF

clear all;
warning('off','all');
addpath('DataFiles');

%Import gait, CCF data, and physics
load('FlapperGait.mat');
load('FlapperCCFData.mat');
load('FlapperMetricCoriolis.mat');

%Spring and damping constants
k1 = .08;
k2 = .3;
b = .01;

rossred = [234 14 30]/255;

%Import best gait fourier params
xs = best;

%Generate motion based off number of fourier params
if numel(xs) == 6
    [p,T] = makeGait1D6(xs);
else
    [p,T] = makeGait1D(xs);
end

%Run simulation for optimal gait
[displ,cost,angles,final_loop] = simulatePassiveSwimmer_bs(p,T,funs,k1,k2,b,0);

%Get total X,Y,Theta displacement from gait
gx = final_loop(1,end);
gy = final_loop(2,end);
gtheta = final_loop(3,end);

%Plot CCF contours
Hy = H{5};
Hpsi = Hy;
figure(13);
clf;
contour(grid{:},Hpsi,7,'linewidth',2);
axis equal;
axis tight;
colormap(Colorset.colormap_contour); 
hold on;

%Change starting phase of gait for arrow generation
phaseShiftPercent = .2;
phaseGrab = floor(size(angles,2)*phaseShiftPercent)+1;
angles = [angles(:,phaseGrab:end),angles(:,1:phaseGrab)];

angles = angles(:,1:101);
angles(:,end) = angles(:,1);

%Plot gait
lw = 3;
plot(angles(2,:),angles(1,:),'Color',rossred,'LineWidth',lw);

%Generate and plot arrowheads for gait directions
heads = plot_dir_arrows(angles(2,:),angles(1,:),2);
for i = 1:numel(heads)
    plot(heads(i).XData,heads(i).YData,'Color',rossred,'LineWidth',lw);
end

%Set plot ticks
xticks([-pi/3,0,pi/3]);
yticks([-pi/3,0,pi/3]);
xticklabels('');
yticklabels('');

warning('on','all');


        