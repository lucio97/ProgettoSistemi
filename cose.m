clear all;
close all;
clc;

radius = 128; %raggio approssimato ricavato convertendo l`area data dal pdf
xx0 = 0;
yy0 = 0;
areaTotale=pi*radius^2; 
h_drone = 320;

lambda=0.001; %area grande lambda piccolo

numbPoints=poissrnd(areaTotale*lambda);%Poisson number of points
theta=2*pi*(rand(numbPoints,1)); %angular coordinates
rho2=radius*sqrt(rand(numbPoints,1)); %radial coordinates
%Convert from polar to Cartesian coordinates
[xx,yy]=pol2cart(theta,rho2); %x/y coordinates of Poisson points
D = pdist2([0 0], [xx, yy]);
D = transpose(D);
C = hypot(D,h_drone);
D = [D,C];
clear C
header = {'Raggio','Distanza'};
xForDisplay = [header; num2cell(D)];
figure
uitable('Data', xForDisplay);
clear xForDisplay
%Shift centre of disk to (xx0,yy0)
xx=xx+xx0;
yy=yy+yy0;
%Plotting
% scatter(xx,yy);
% xlabel('x');ylabel('y');
% axis square;



figure
pax = polaraxes;

polarplot(theta,rho2,'d')

pax.ThetaDir = 'counterclockwise';
pax.FontSize = 12;
sz=size(D,1);
