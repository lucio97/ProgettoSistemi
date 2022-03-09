clear all;
close all;
clc;
lambda=1e-2;
i=100;
while 1
    test=programma(i,lambda);
    ciao=[i,test];
    disp(ciao)
    i=i-1;
    if i==0
        break
    end
    
end