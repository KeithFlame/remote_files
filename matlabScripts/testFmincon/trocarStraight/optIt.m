%% init value
xhmax0=[1 1 1 1 1 1];
xhmin0=[-1 -1 0 -1 0 -1];
num=1;
initV=zeros(size(xhmax0,2),num);
result_k=zeros(size(xhmax0,2),num);
exitflag_k0_b=zeros(1,num);
residual_k0=zeros(1,num);
x0=zeros(max(size(xhmax0)),1);
% x0=[0.0054    0.4993    0.0289    0.0021   -0.0000    0.0035]';  % 仅本身
% x0=[0.1187    0.0009    0.0237   -0.1255    0.0000   -0.1184]'; % 加上base_dev
for i =1 :num
%     x0=rand(size(xhmax0,2),1).*(xhmax0'-xhmin0')+xhmin0';
    [xh0,yh0,exitFlag_k0]=fmincon('costFunc_clearance_psi0',x0,[],[],[],[],xhmin0,xhmax0,[],options);
    residual_k0(i)=yh0;
    exitflag_k0_b(i)=exitFlag_k0;
    result_k(:,i)=xh0;
    initV(:,i)=x0;

end
xh0(2)=xh0(2)/1000;
psi0=xh0';
