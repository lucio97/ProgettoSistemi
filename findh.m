i=1;
while 1
    test=programma(i);
    ciao=[i,test];
    disp(ciao)
    if test>0.5 & test< 1.5
        break
    else
        i=i+1;
    end
end