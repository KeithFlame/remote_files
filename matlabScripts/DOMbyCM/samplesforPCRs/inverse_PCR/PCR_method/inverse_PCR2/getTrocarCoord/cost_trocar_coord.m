function f=cost_trocar_coord(x)

psi_block=[
    % arm1
0 60 0 0 0 0
0 80 0 0 0 0
0 60 30 -90 0 0
0 60 0 0 30 -90
0 60 0 0 30 180
0 60 0 0 30 0
0 100 30 -90 0 0
0 100 0 0 30 -90
0 100 0 0 30 180
0 100 0 0 30 0
%arm2
% % 0 100 0 0 0 0
% % 0 120 0 0 0 0
% % 0 120 0 0 30 90
% % 0 120 30 90 0 0
% % 0 120 30 90 30 -90
% % 0 120 45 90 0 0
% % 0 120 15 0 30 180
% % 0 120 45 90 50 -90
% % 0 120 30 0 50 180
% % 45 120 30 0 50 180
]'*pi/180;
psi_block(2,:) = psi_block(2,:)*180/pi;
Lg=x(7);
x_truth=getTruth(psi_block,Lg);
num = size(psi_block,2);
x_cur=getCur;
x_res = zeros(num,6);
cTa1=fromX2T(x(1:6));
for i = 1:num
    xt=x_truth(:,i);
    xc=x_cur(:,i);    
    a1Tend=fromX2T(xt);
    Tc = fromX2T(xc);
    dT=abs(calcDeviationByT(cTa1*a1Tend,Tc));
    x_res(i,:) = dT';
end
f = sum(mean(x_res));