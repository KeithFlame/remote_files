options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
% x0=zeros(6,1)+1;x0(3)=-150;x0(6)=1;
% x0=[1.5454    0.5120 -138.9055 -517.9033 -735.3975   -5.3223]';
x0 =rand(6,1);x0(3)=x0(3)-150;x0(6)=1;
x0=[-28.7841   -4.4863  -32.9935  -90.7323    1.0003   -6.8320]';
% x0=[-22.8351    7.7388    2.9139  -87.1706   28.8594  -27.7343]';
[x,fval,exitflag,output] = fmincon('costFunc_CoordHK_2',x0,[],[],[],[],[],[],[],options);