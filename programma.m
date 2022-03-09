function EmilioMin = programma(radius, lambda)

% I commenti in inglese sono seri, quelli in italiano un po' (tanto) meno



% % Variables
% radius = 2000; %m approximated found by the given area on the pdf
xx0 = 0;
yy0 = 0;
h_drone=320;
areaTotale=pi*radius^2; 
h_ric=0;
G_tx = 100; 
G_tx_dB = 10*log10(G_tx);
G_rx = 1;
G_rx_dB = 10*log10(G_rx);
B_signal=2*10^7;
freq = 2.4*10^9;
c = physconst('lightspeed');
wavelenght= c/freq;
P_tx = 0.063; %18 dbm
P_tx_dB = 10*log10(P_tx);
P_N = 2;  %3 dB
a = 0.3;
b =300e-6; % buildings/m^2
% lambda=1; % u/m big area little lambda
eta_l=2;
eta_nl=3;
crowns=20;
crowns_radius= radius/crowns;
power_percent=100/crowns;
cd=2*radius-(radius/4);
xd= [-cd 0 0 cd];
yd= [0 -cd cd 0];
xd = transpose(xd);
yd = transpose(yd);


%Main
numbPoints=poissrnd(areaTotale*lambda); %Poisson number of receiver
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

theta=2*pi*(rand(numbPoints,1)); %angular coordinates for plot
rho2=radius*sqrt(rand(numbPoints,1)); %radial coordinates

%Convert from polar to Cartesian coordinates
[x,y]=pol2cart(theta,rho2); %x/y coordinates of Poisson points
D = pdist2([xx0 yy0], [x, y]);
D = transpose(D); %^raggio
C = hypot(D,h_drone);
E = asin(h_drone./C); %elevation angle
F = asind(h_drone./C);

phi_3dB=70;
A_m=25; %dB
theta_3dB=10;
SLA_v=20;

A_h=-min((12.*((90-F)./phi_3dB).^2), A_m);
Theta=90-F;
theta_b=90;
A_v=-min((12.*((Theta-theta_b)./theta_3dB).^2), SLA_v);
A_fin=-min((-A_h-A_v), A_m);
G_tfin=A_fin+G_tx_dB;

%hyp: decreased power over distance 
%creation of 20 crowns, more distance more dB decreased
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
end
% clear i k rangeinf rangesup
G = transpose(G);

D = [D,C,E,F,G];
clear C E F G
% figure('unit','normalized', 'position',[0.1 0.1 0.5 0.5])
% uitable('Data', D, 'columnname', {'Raggio','Distanza','Theta in Radianti','Theta in Gradi','Antenna gain %'},'unit','normalized', 'Position', [0 0 1 1]);
%Shift centre of circle to (xx0,yy0)
x=x+xx0;
y=y+yy0;

m=floor(D(:,2).*sqrt(a*b));
prob_los=zeros(numbPoints,1);
for i=1:numbPoints
    plostmp=1;
    for k=0:m(i)
        plostmp1=1-exp(-((((h_drone-(k+0.5)*(h_drone+h_ric))/(m(i)+1))^2)/(2*(15^2))));
        plostmp=plostmp1*plostmp;
    end
    prob_los(i)=plostmp;
end
clear plostmp plostmp1 tmp i k m

%freezer dragon ball
Xlos = 1 + 2.88.*randn(numbPoints,1);
Xnlos = 1 + 10.*randn(numbPoints,1);
pl_los=(20*log10((4*pi)/wavelenght))+(10*eta_l*log10(D(:,2)))+Xlos;
pl_nlos=(20*log10((4*pi)/wavelenght))+(10*eta_nl*log10(D(:,2)))+Xnlos;
path_loss=prob_los.*pl_los+((1-prob_los).*pl_nlos); %Average path loss w/o considering gains
P_rx = P_tx*G_tx*G_rx*(wavelenght./(4*pi*D(:,2))).^2;
path_loss_lin=10.^(path_loss./10);
CrownsGain_tx_hyp=(G_tx_dB.*(D(:,5)./100)); 
P_rx_pulita=P_tx_dB-path_loss+G_tfin+G_rx_dB;
P_rx_pulita_lin=10.^(P_rx_pulita./10);
SNR = P_rx_pulita_lin/P_N;
Capacity=B_signal*log2(SNR+1);
mediaP_rx = mean(P_rx);

%display stations in two different images
% figure('Name','Plots','NumberTitle','off','WindowState','maximized')
% subplot(1,2,1)
% polarplot(theta,rho2,'d')
% rticks([crowns_radius 2*crowns_radius 3*crowns_radius 4*crowns_radius 5*crowns_radius 6*crowns_radius  7*crowns_radius 8*crowns_radius 9*crowns_radius 10*crowns_radius 11*crowns_radius 12*crowns_radius 13*crowns_radius 14*crowns_radius 15*crowns_radius 16*crowns_radius 17*crowns_radius 18*crowns_radius 19*crowns_radius 20*crowns_radius])
% rticklabels({'', '', '', '', '500m', '', '', '', '', '1000m', '', '', '', '', '1500m', '', '1700m', '', '', '2000m'})
% subplot(1,2,2)
% for i=1:size(xd,1)
%     circle(xd(i),yd(i),radius);
% end
% circle(0,0,radius);
% hold on
% scatter(xx,yy,'d');
% scatter(xd,yd,100,'p', 'filled','red');
% scatter(0,0,100,'p', 'filled','red');
% cmap = hsv(11);
% gscatter(x,y,prob_los,cmap);
% hold off
CoordExt = [xx yy];
clear i xx yy

SIR=zeros(numbPoints,1);
%  SIR and SINR computation
for i=1:numbPoints
    sommp=0;
    sommpl=0;
    sommtmp=0;
    sommtot=0;
    for k=1:size(xd,1)
        dtmp = pdist2([x(i), y(i)], [xd(k), yd(k)]);
        if dtmp<radius
            dtmp = hypot(dtmp,h_drone);
            sommp=prob_los(i)*dtmp^(-eta_l);
            sommpl=(1-prob_los(i))*dtmp^(-eta_nl);
            sommtmp=sommp+sommpl;
        end
        sommtot=sommtot+sommtmp;
    end
    if sommtot==0
        SIR(i,1)=NaN;
    else
        SIR(i,1)=(((prob_los(i)*(D(i,2)))^(-eta_l))+((1-prob_los(i))*(D(i,2)))^(-eta_nl))/(sommtot);
    end
end
clear somm dtmp i k sommp sommpl sommtmp sommtot
SINR = ((SNR.*SIR)./(SNR+SIR));

% SINR_threshold
P_rx_threshold = 5e-13;
count=0;
for i=1:numbPoints
    if P_rx_pulita_lin(i)>=P_rx_threshold
        count=count+1;
    end
end
Prob_threshold=count/numbPoints;
polarfun = @(theta,r) r*Prob_threshold;
Coverage = (1/(pi*radius^2))*integral2(polarfun,0,2*pi,0,radius);

clear i count Prob_threshold polarfun ans

% UpLink/ in salita emilio matricciani vibes

freq_up=2*10^9;
wavelenght_up=c/freq_up;
P_rx_up = P_tx*G_tx*G_rx*(wavelenght_up./(4*pi.*D(:,2))).^2;
P_rx_pulita_up=P_tx_dB-path_loss+G_tx_dB+G_rx_dB; % hyp on-ground devices look at the drone
P_rx_pulita_lin_up=10.^(P_rx_pulita_up./10);
SNR_up= P_rx_pulita_lin_up/P_N;
% SIR_up=1/numbPoints; % perchè lo dice savino/ distance devices ground-drone is the same as the interfeering devices w/ drone. devices look at the drone
% SIR =Prx/PI=d−η/∑li=1r−ηi
% d=dist d:2
% r= distanza drone punto esterno
DistExt = hypot(transpose(pdist2([xx0, yy0], [CoordExt(:,1), CoordExt(:,2)])),h_drone);
m=floor(DistExt.*sqrt(a*b));
prob_los_ext=zeros(size(CoordExt,1),1);
for i=1:size(CoordExt,1)
    plostmp=1;
    for k=0:m(i)
        plostmp1=1-exp(-((((h_drone-(k+0.5)*(h_drone+h_ric))/(m(i)+1))^2)/(2*(15^2))));
        plostmp=plostmp1*plostmp;
    end
    prob_los_ext(i,1)=plostmp;
end

% SIR_up=zeros(size(CoordExt,1),1);
for i=1:size(CoordExt,1)
    sommp=prob_los_ext(i)*DistExt(i)^(-eta_l);
    sommpl=(1-prob_los_ext(i))*DistExt(i)^(-eta_nl);
    sommt=sommp+sommpl;    
end
SIR_up=(((prob_los.*(D(:,2))).^(-eta_l))+((1-prob_los).*(D(:,2))).^(-eta_nl))/sommt;
SINR_up=(SNR_up.*SIR_up)./(SNR_up+SIR_up);
Capacity_up=B_signal*log2(SNR_up+1);
clear  i sommp sommpl sommt

% mean sinr/sir
media_SIR = mean(SIR,'omitnan');
media_SINR = mean(SINR,'omitnan');
media_SIR_up = mean(SIR_up,'omitnan');
media_SINR_up = mean(SINR_up,'omitnan');

% outage probability
pr_outage_threshold=10^-16;
count=0;
for i=1:numbPoints
    if P_rx_pulita_lin(i)<pr_outage_threshold
        count=count+1;
    end
end
pr_outage=(count/numbPoints)*100; %*100 perchè leo voleva la percentuale se no non era contento
clear i count pr_outage_threshold
 
P_rx_pulita_lin_hyp=P_tx*G_rx./path_loss_lin.*CrownsGain_tx_hyp;
Emilio=P_rx_pulita_lin_hyp./P_rx_pulita_lin;
EmilioMin=min(Emilio);
EmilioMax=max(Emilio);
EmilioMean=mean(Emilio);

% find drone altitude so that EmilioMean is ~1 


% figure
% [X,Y]=meshgrid(x,y);
% Z=zeros(size(x));
% Z=repmat(Z,1,numbPoints);
% plot3(x,y,Z,'d');
% hold on
% xlabel('x');
% ylabel('y');
% zlabel('z');
% img = imread('drone.png');% Load a sampleimage
% xImage = [-500 500; -500 500]; % The x data for the image corners
% yImage = [0 0; 0 0]; % The y data for the image corners
% zImage = [1000 1000; 500 500]; % The z data for the image corners
% surf(xImage,yImage,zImage,'CData',img,'FaceColor','texturemap'); % Plot the surface