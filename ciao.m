clear all;
close all;
clc;
for i=1:3
    if i==1
        SNR1=SNR_medio(220);
    elseif i==2
        SNR2=SNR_medio(240);
    else
        SNR3=SNR_medio(260);
    end
end
SNR=[SNR1,SNR2,SNR3];