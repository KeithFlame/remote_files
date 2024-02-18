function T = getData_CW

T=zeros(6,80);
marker_ID1 = 'SBM4';
marker_ID2 = 'SBM14';
marker_ID3 = 'SBM40';
[Tt4, Tt14, Tt40] = registration_CW3;
for i = 1:80
    file = ['P',num2str(i)];
    [C4, C14, C40,marker_ID]=getData71931(file,2);
    if(strcmp(marker_ID1,marker_ID))
        T4 = getCoord(C4);
        Tt = T4/Tt4;
    elseif(strcmp(marker_ID2,marker_ID))
        T14 = getCoord(C14);
        Tt = T14/Tt14;
    elseif(strcmp(marker_ID3,marker_ID))
        T40 = getCoord(C40);
        Tt = T40/Tt40;
    end
    X = fromT2X(Tt);
    T(:,i)=X;
end
end