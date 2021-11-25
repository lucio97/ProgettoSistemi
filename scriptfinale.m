clear all;
close all;
clc;


radius = 128; %raggio approssimato ricavato convertendo l`area data dal pdf
xx0 = 0;
yy0 = 0;
areaTotale=pi*radius^2; 
h_drone = 320;
h_ric=0;
G_tx = 4;
G_rx = 2;
freq = 5800;
c = physconst('lightspeed');
wavelenght= c/freq;
P_tx = 0.063;
a = 0.3;
b =300*10^(-6);


lambda=0.001; %area grande lambda piccolo

numbPoints=poissrnd(areaTotale*lambda);%Poisson number of points
theta=2*pi*(rand(numbPoints,1)); %angular coordinates
rho2=radius*sqrt(rand(numbPoints,1)); %radial coordinates
%Convert from polar to Cartesian coordinates
[xx,yy]=pol2cart(theta,rho2); %x/y coordinates of Poisson points
D = pdist2([0 0], [xx, yy]);
D = transpose(D);
C = hypot(D,h_drone);
E = asin(h_drone./C); 
F = asind(h_drone./C);
% D = [D,C,E,F];
D = [D,C,F];

% clear C E F
clear C F
% header = {'Raggio','Distanza','Theta in Radianti','Theta in Gradi'};
header = {'Raggio','Distanza','Theta in Gradi'};
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

% x = [x; newval]
%plos=1./(1+a.*exp(-b.*(E-a)));
m=floor(D(:,1).*sqrt(a*b));
count=size(m,1);
plos=zeros(count,1);
for i=1:count
    tmp=m(i);
    plostmp=1;
    for k=0:tmp
    plostmp1=1-exp(-((((h_drone-(k+0.5)*(h_drone+h_ric))/(tmp+1))^2)/(2*(15^2))));
    plostmp=plostmp1*plostmp;
    end
    plos(i)=plostmp;
end
clear plostmp plostmp1

% 
% P_rx = P_tx*G_tx*G_rx*(wavelenght/4*pi*D(:,2)).^2;
% mediaP_rx = mean(P_rx);


figure
pax = polaraxes;

polarplot(theta,rho2,'d')

pax.ThetaDir = 'counterclockwise';
pax.FontSize = 12;