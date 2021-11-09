function [origin,cter]=getOriginV2(data_Origin)
global Pvy;

Pvy=data_Origin(:,1:3);



options = optimoptions('fmincon','Algorithm','interior-point'); % 
options.StepTolerance=1e-20;
[x1,y0,exitflag]=fmincon('costFuncToGetY',[0 1 0],[],[],[],[],[],[],[],options);
y0=y0/1000;
origin=x1';
cter=y0;
end