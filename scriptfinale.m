clear all;
close all;
clc;


%nanmean per media sinr/sir

% to do
% uplink e prob. di fuori servizio con solgia si SINR
% 

% % Variables
radius = 2000; %m approximated found by the given area on the pdf
xx0 = 0;
yy0 = 0;
areaTotale=pi*radius^2; 
h_drone = 320;
h_ric=0;
G_tx = 100; 
G_tx_dB = 10*log10(G_tx);
G_rx = 1;
G_rx_dB = 10*log10(G_rx);
freq = 2.4*10^9;
c = physconst('lightspeed');
wavelenght= c/freq;
P_tx = 0.063; %18 dbm
P_tx_dB = 10*log10(P_tx);
P_N = 2;  %3 dB
a = 0.3;
b =300e-6; % buildings/m^2
lambda=1e-5; % u/m big area little lambda
eta_l=2;
eta_nl=3;
crowns=20;
crowns_radius= radius/crowns;
power_percent=100/crowns;
cd=3500;
xd= [-cd 0 0 cd];
yd= [0 -cd cd 0];
xd = transpose(xd);
yd = transpose(yd);


%Main
numbPoints=poissrnd(areaTotale*lambda);
xx=[];
yy=[];
for i=1:size(xd,1)
    theta=2*pi*(rand(numbPoints,1));
    rho2=radius*sqrt(rand(numbPoints,1));
    [x,y]=pol2cart(theta,rho2);
    xtmp=x-xd(i);
    ytmp=y-yd(i);
    xx= vertcat(xx,xtmp);
    yy= vertcat(yy,ytmp);
    numbPoints=poissrnd(areaTotale*lambda);
end
clear xtmp ytmp i

% numbPoints=poissrnd(areaTotale*lambda);%Poisson number of receiver
theta=2*pi*(rand(numbPoints,1)); %angular coordinates for plot
rho2=radius*sqrt(rand(numbPoints,1)); %radial coordinates
%Convert from polar to Cartesian coordinates
[x,y]=pol2cart(theta,rho2); %x/y coordinates of Poisson points
D = pdist2([0 0], [x, y]);
D = transpose(D); %^raggio
C = hypot(D,h_drone);
E = asin(h_drone./C); %elevation angle
F = asind(h_drone./C);

phi_3dB=70; %gradi ho messo la b grande
A_m=25; %sono decibel
theta_3dB=10; %
SLA_v=20;

A_h=-min((12.*((90-F)./phi_3dB).^2), A_m);
Theta=90-F;
theta_b=90;
A_v=-min((12.*((Theta-theta_b)./theta_3dB).^2), SLA_v); %da finire

for i=1:numbPoints
    rangeinf=0;
    rangesup=crowns_radius;
    for k=1:crowns
        if (D(i)>=rangeinf) & (D(i)<rangesup)
            G(i)=(100-((k-1)*power_percent));
            break
        elseif k==crowns
            disp('Error in power calculation');
        else
            rangeinf=rangeinf+crowns_radius;
            rangesup=rangesup+crowns_radius;
        end
    end
% 
end
clear i k rangeinf rangesup
G = transpose(G);

D = [D,C,E,F,G];
clear C E F G
% D = [D,C,F];
% clear C F
figure('unit','normalized', 'position',[0.1 0.1 0.5 0.5])
uitable('Data', D, 'columnname', {'Raggio','Distanza','Theta in Radianti','Theta in Gradi','Antenna gain %'},'unit','normalized', 'Position', [0 0 1 1]);
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

%freezer dragon ball
Xlos = 1 + 2.88.*randn(numbPoints,1);
Xnlos = 1 + 10.*randn(numbPoints,1);
pl_los=(20*log10((4*pi)/wavelenght))+(10*eta_l*log10(D(:,2)))+Xlos;
pl_nlos=(20*log10((4*pi)/wavelenght))+(10*eta_nl*log10(D(:,2)))+Xnlos;
path_loss=prob_los.*pl_los+((1-prob_los).*pl_nlos);
P_rx = P_tx*G_tx*G_rx*(wavelenght/4*pi*D(:,2)).^2;
% mediaP_rx = mean(P_rx);
SNR = P_tx/P_N;

% creare ca. 20 corone in cui si perdono dB man mano che ci si allontana
% dal centro

path_loss_lin=10.^(path_loss./10);
% proval=db2mag(path_loss);
% P_rx_pulita=P_tx-path_loss_lin;
prova=(G_tx_dB.*(D(:,5)./100));
P_rx_pulita=P_tx_dB-path_loss+prova+G_rx_dB;
P_rx_pulita_lin=10.^(P_rx_pulita./10);

figure('Name','Plots','NumberTitle','off','WindowState','maximized')
subplot(1,2,1)
% pax = polaraxes;
polarplot(theta,rho2,'d')
% rticks([crowns_radius 2*crowns_radius 3*crowns_radius 4*crowns_radius 5*crowns_radius 6*crowns_radius  7*crowns_radius 8*crowns_radius 9*crowns_radius 10*crowns_radius 11*crowns_radius 12*crowns_radius 13*crowns_radius 14*crowns_radius 15*crowns_radius 16*crowns_radius 17*crowns_radius 18*crowns_radius 19*crowns_radius 20*crowns_radius])
% rticklabels({'', '', '', '', 500, '', '', '', '', 1000, '', '', '', '', 1500, '', 1700, '', '', 2000})
% pax.ThetaDir = 'counterclockwise';
% pax.FontSize = 12;
subplot(1,2,2)
for i=1:size(xd,1)
    circle(xd(i),yd(i),radius);
end
circle(0,0,radius);
hold on
scatter(xx,yy,'d');
scatter(xd,yd,100,'p', 'filled','red');
scatter(0,0,100,'p', 'filled','red');
cmap = hsv(11);
gscatter(x,y,prob_los,cmap);
% colormap(sort(cmap, 'descend'));
% colorbar;
hold off
clear i xx yy

SIR=zeros(numbPoints,1);
for i=1:numbPoints
%     somm=1;
    sommp=1;
    sommpl=1;
    for k=1:size(xd,1)
        dtmp = pdist2([x(i), y(i)], [xd(k), yd(k)]);
        if dtmp<radius
            dtmp = hypot(dtmp,h_drone);
%             dtmp = dtmp^(-eta_nl);
%             somm=somm*dtmp;
            sommp=prob_los(i)*dtmp^(-eta_l);
            sommpl=(1-prob_los(i))*dtmp^(-eta_nl);
        end
    end
%     if somm==1
%         SIR(i,1)=NaN;
%     else
%         SIR(i,1)=((D(i,2))^(-eta_nl)/somm);
%     end    
    if sommp==1 && sommpl==1
        SIR(i,1)=NaN;
    else
        SIR(i,1)=(((prob_los(i)*(D(i,2)))^(-eta_l))+((1-prob_los(i))*(D(i,2)))^(-eta_nl))/(sommp+sommpl);
    end
end
clear somm dtmp i k sommp sommpl
SINR = ((SNR.*SIR)./(SNR+SIR));

% SINR_soglia
P_rx_soglia = 5e-13;
count=0;
for i=1:numbPoints
    if P_rx_pulita_lin(i)>=P_rx_soglia
        count=count+1;
    end
end
Prob_soglia=count/numbPoints;
polarfun = @(theta,r) r*Prob_soglia;
Coverage = (1/(pi*radius^2))*integral2(polarfun,0,2*pi,0,radius);

clear i count Prob_soglia polarfun ans
 

% figure
% [X,Y]=meshgrid(x,y);
% Z=zeros(size(x));
% Z=repmat(Z,1,numbPoints);
% surf(X,Y,Z)