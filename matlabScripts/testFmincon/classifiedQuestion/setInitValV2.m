function SP_res=setInitValV2(SP)
%
%
%
%
%%
persistent SP1;
if(isempty(SP1))
    %there is a stupid man.

end
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
SP.structure.K1=[    0.0191         0         0
         0    0.0191         0
         0         0    0.0153];
SP.structure.zeta=0.1994;
SP.structure.Ks=SP.structure.K1/SP.structure.zeta;

%% trocar
% SP.trocar.arc_radius=200;
SP.trocar.c=0.51e-3;
SP.trocar.arc_length=0.1;
SP.trocar.line_2_length=0.074;

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
theta1_b=L1/(SP.trocar.line_2_length*SP.structure.zeta+L1)*SP.psi.theta1;
thetas_b=SP.trocar.line_2_length*SP.structure.zeta/(SP.trocar.line_2_length*SP.structure.zeta+L1)*SP.psi.theta1;
u1_b=[0 theta1_b/L1 0]';
us_b=[0 thetas_b/SP.trocar.line_2_length 0]';
SP.dependent_psi.u1_b=u1_b;
SP.dependent_psi.us_b=us_b;
%% result
SP1=SP;
SP_res=SP1;




end

