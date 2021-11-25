M = 5;
rho = 1.2;
g = 9.8;
diametro_eliche = 0.38;           %circa 15 inches di diametro per elica
A = pi*pow(diametro_eliche/2, 2);
h_drone = 320;
lambda = 1;
x = 1;
P_tx = 0.063;    %18 dBm
P_tx_db = 10*log10(P_tx);
P_N_db = 3;
P_treshold = 10^-15;

Poisson = pow(e, -lambda)*pow(lambda, x)/factorial(x);

P_drone = sqrt((pow(2*M*g, 3)/(16*rho*A));

P_consumata = P_tx + P_drone;

SNR = P_tx/P_N;

SNR_db = P_tx_db - P_N_db;