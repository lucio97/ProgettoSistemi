clear all;
close all;
clc;

% to do
% altri droni (3.5km) ok
% distanza droni ricev ok
% formula sinr e sir ok
% uplink e downlink

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
P_N = 2;
a = 0.3;
b =300e-6; % buildings/m^2
lambda=5e-6; % u/m big area little lambda
eta_l=2;
eta_nl=3;
xd= [-3500 0 0 3500];
yd= [0 -3500 3500 0];
xd = transpose(xd);
yd = transpose(yd);


%Main
numbPoints=poissrnd(areaTotale*lambda);%Poisson number of receiver
theta=2*pi*(rand(numbPoints,1)); %angular coordinates for plot
rho2=radius*sqrt(rand(numbPoints,1)); %radial coordinates
%Convert from polar to Cartesian coordinates
[x,y]=pol2cart(theta,rho2); %x/y coordinates of Poisson points
D = pdist2([0 0], [x, y]);
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
x=x+xx0;
y=y+yy0;

m=floor(D(:,2).*sqrt(a*b));
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
P_rx = P_tx*G_tx*G_rx*(wavelenght/4*pi*D(:,2)).^2;
% mediaP_rx = mean(P_rx);
SNR = P_tx/P_N;

figure
subplot(1,2,1)
% pax = polaraxes;
polarplot(theta,rho2,'d')
% pax.ThetaDir = 'counterclockwise';
% pax.FontSize = 12;
subplot(1,2,2)
% figure
for i=1:size(xd,1)
    circle(xd(i),yd(i),radius);
end
circle(0,0,radius);
hold on
gscatter(x,y,prob_los);
hold off
clear i,

SIR=zeros(numbPoints,1);
for i=1:numbPoints
    somm=1;
    for k=1:size(xd,1)
        dtmp = pdist2([x(i), y(i)], [xd(k), yd(k)]);
        if dtmp<radius
            dtmp = hypot(dtmp,h_drone);
            dtmp = dtmp^(-eta_nl);
            somm=somm*dtmp;
        end
    end
    if somm==1
        SIR(i,1)=0;
    else
        SIR(i,1)=((D(i,2))^(-eta_nl)/somm);
    end
end
clear somm dtmp i k
SINR = ((SNR.*SIR)./(SNR+SIR));

% figure
% [X,Y]=meshgrid(x,y);
% Z=zeros(size(x));
% Z=repmat(Z,1,numbPoints);
% surf(X,Y,Z)
