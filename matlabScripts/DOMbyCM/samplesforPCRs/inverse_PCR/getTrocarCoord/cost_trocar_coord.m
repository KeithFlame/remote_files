function f=cost_trocar_coord(x)

psi_block=[
0 100 0 0 0 0    
]';
x_truth=getTruth(psi_block);
num = size(psi_block,2);
x_cur=getCur;
x_res = zeros(num,6);
for i = 1:num
    xt=x_truth(:,i);
    xc=x_cur(:,i);
    cTa1=fromX2T(x);
    a1Tend=fromX2T(xt);
    Tc = fromX2T(xc);
    dT=abs(calcDeviationByT(cTa1*a1Tend,Tc));
    x_res(i,:) = dT';
end
f = sum(sum(x_res));