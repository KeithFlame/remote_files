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

%% structure
[~, L1, ~, Lr, ~, L2, ~, Lg, ~,~,Lstem]=getToolArmStructureParameter;
SP.structure.L2=L2;
SP.structure.Lr=Lr;
SP.structure.Lg=Lg;
SP.structure.L1=L1;
SP.structure.Lstem=Lstem;
SP.structure.K1=[    0.0190         0         0
         0    0.0190         0
         0         0    0.0152];
SP.structure.zeta=0.1;
SP.structure.Ks=SP.structure.K1/SP.structure.zeta;

%% dependent varibale
if(SP.psi.l<SP.structure.Lr+SP.structure.L2)
    SP.psi.l=SP.structure.Lr+SP.structure.L2;
end
if(SP.psi.l-SP.structure.L1-SP.structure.L2-SP.structure.Lr>0)
    SP.dependent_psi.Ls=SP.psi.l-SP.structure.L1-SP.structure.L2-SP.structure.Lr;
else
    SP.dependent_psi.Ls=0;
end

SP.dependent_psi.delta1i=SP.psi.delta1;
SP.dependent_psi.deltasi=SP.psi.delta1;
SP.dependent_psi.delta1o=SP.psi.delta1;
SP.dependent_psi.deltaso=SP.psi.delta1;
%% trocar
SP.trocar.arc_radius=200;
SP.trocar.c=0.5e-3;
SP.trocar.arc_length=0.1;
%% result
SP1=SP;
SP_res=SP1;
end

