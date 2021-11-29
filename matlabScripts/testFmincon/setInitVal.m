function SP_res=setInitVal(SP)
%
%
%
%
%%
persistent SP1;
if(nargin==0)
    SP_res=SP1;
    return;
end

[~, L1, ~, Lr, ~, L2, ~, Lg, ~,~,Lstem]=getToolArmStructureParameter;
SP.L2=L2;
SP.Lr=Lr;
SP.Lg=Lg;
SP.L1=L1;
SP.Lstem=Lstem;
SP.K1=[    0.0190         0         0
         0    0.0190         0
         0         0    0.0152];

SP.c=0.3e-3;
if(SP.l<SP.Lr+SP.L2)
    SP.l=SP.Lr+SP.L2;
end
if(SP.l-SP.L1-SP.L2-SP.Lr>0)
    SP.Ls=SP.l-SP.L1-SP.L2-SP.Lr;
else
    SP.Ls=0;
end
% SP.zeta=SP.K1(1,1)/SP.Ks(1,1);
SP.zeta=0.1;
SP.Ks=SP.K1/SP.zeta;
SP1=SP;
SP_res=SP1;
end

