clear all;
close all;
clc;

% % Variables
radius = 2000; %m approximated found by the given area on the pdf
xx0 = 0;
yy0 = 0;
areaTotale=pi*radius^2; 
h_drone = 1000;
h_ric=0;
G_tx = 4;
G_rx = 2;
freq = 2.4*10^9;
c = physconst('lightspeed');
wavelenght= c/freq;
P_tx = 0.063; % dbm
a = 0.3;
b =300e-6; % buildings/m^2
lambda=1e-5; % u/m big area little lambda
eta_l=2;
eta_nl=3;

%Main
numbPoints=poissrnd(areaTotale*lambda);%Poisson number of receiver
theta=2*pi*(rand(numbPoints,1)); %angular coordinates for plot
rho2=radius*sqrt(rand(numbPoints,1)); %radial coordinates
%Convert from polar to Cartesian coordinates
[xx,yy]=pol2cart(theta,rho2); %x/y coordinates of Poisson points
D = pdist2([0 0], [xx, yy]);
D = transpose(D);
C = hypot(D,h_drone);
ThetaRad = asin(h_drone./C); %elevation angle
F = asind(h_drone./C);
% D = [D,C,E,F];
% clear C E F
% header = {'Raggio','Distanza','Theta in Radianti','Theta in Gradi'};
D = [D,C,F];
clear C F
header = {'Raggio','Distanza','Theta in Gradi'};
xForDisplay = [header; num2cell(D)];
figure
uitable('Data', xForDisplay);
clear xForDisplay header
%Shift centre of disk to (xx0,yy0)
xx=xx+xx0;
yy=yy+yy0;



m=floor(D(:,2).*sqrt(a*b));
numbPoints=size(m,1);
prob_los=zeros(numbPoints,1);
for i=1:numbPoints
    tmp=m(i);
    plostmp=1;
    for k=0:tmp
    plostmp1=1-exp(-((((h_drone-(k+0.5)*(h_drone+h_ric))/(tmp+1))^2)/(2*(15^2))));
    plostmp=plostmp1*plostmp;
    end
    prob_los(i)=plostmp;
end
clear plostmp plostmp1 tmp i k

Xlos = 1 + 2.88.*randn(numbPoints,1);
Xnlos = 1 + 10.*randn(numbPoints,1);
pl_los=(20*log10((4*pi)/wavelenght))+(10*eta_l*log10(D(:,2)))+Xlos;
pl_nlos=(20*log10((4*pi)/wavelenght))+(10*eta_nl*log10(D(:,2)))+Xnlos;
path_loss=prob_los.*pl_los+((1-prob_los).*pl_nlos);



% P_rx = P_tx*G_tx*G_rx*(wavelenght/4*pi*D(:,2)).^2;
% mediaP_rx = mean(P_rx);


figure
pax = polaraxes;

polarplot(theta,rho2,'d')

pax.ThetaDir = 'counterclockwise';
pax.FontSize = 12;