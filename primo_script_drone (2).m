% 
% M = 5;
% rho = 1.2;
% g = 9.81;
% diametro_eliche = 0.5    ;       %circa 15 inches di diametro per elica
% A = pi*(diametro_eliche/2)^2;
% h_drone = 320;
% n = 4;
% x=1;
% P_tx = 0.063;    %18 dBm in Watt
% P_tx_db = 10*log10(P_tx);
% P_N_db = 3;
% P_treshold = 10^-15;
% 
% p = poisspdf(x,lambda);
% 
% P_drone = sqrt((pow(2*M*g, 3)/(16*rho*n*A)));
% 
% P_consumata = P_tx + P_drone;
% 
% SNR = P_tx/P_N;
% 
% SNR_db = P_tx_db - P_N_db;

%%
radius = 1;
xx0 = 0;
yy0 = 0;
areaTotale=pi*radius^2; 

lambda=10; 

numbPoints=poissrnd(areaTotale*lambda);%Poisson number of points
theta=2*pi*(rand(numbPoints,1)); %angular coordinates
rho2=radius*sqrt(rand(numbPoints,1)); %radial coordinates
%Convert from polar to Cartesian coordinates
[xx,yy]=pol2cart(theta,rho2); %x/y coordinates of Poisson points
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