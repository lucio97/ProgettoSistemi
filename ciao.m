clear all;
close all;
clc;
PathName='./Screens/MeSonCacatoErCazzo/dc.mat';
load(PathName,'numbPoints');
SNR=zeros(numbPoints,1);
for i=20:10:400
   temp=SNR_medio(i,PathName);
   SNR=[SNR, temp];
end
SNR(:,1) = [];