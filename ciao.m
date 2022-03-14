clear all;
close all;
clc;
SNR=zeros(514,1);
for i=200:10:400
   temp=SNR_medio(i);
   SNR=[SNR; temp];
end
%SNR(:,1) = [];