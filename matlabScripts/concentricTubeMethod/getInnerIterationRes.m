function delta_u=getInnerIterationRes(u,P,R,ds)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.18.2021
% Ver. 1.0
% main
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

u0=u;P0=P;R0=R;ds0=ds;
calcInnerCirculationAllVaribale(u,P,R,ds,u0,P0,R0,ds0);
delta=1e-3;
cter=1;
block_size=max(size(u));

d_last=ones(block_size,1);
ddu=zeros(block_size,1);
lamda=ones(block_size,1);
while(cter>1e-9)
    du=getDeltaU(lamda);
    d=getd;
    lamda=lamda+d*delta;
    cter=abs(d_last-d);
    d_last=d;
    ddu=du+ddu;
end
delta_u=ddu;
end
