
function [g,h]=getGH(x)

SP=getSP(x);
thetasi=SP.dependent_psi.thetasi;
theta1i=SP.dependent_psi.theta1i;
lsi=SP.dependent_psi.lsi;
l1i=SP.dependent_psi.l1i;
d=SP.trocar.d;

c=SP.trocar.c;
delta1i=SP.dependent_psi.delta1i;
deltasi=SP.dependent_psi.deltasi;
T_d=getTrocarPose1(d);
if(lsi<1e-4)
    T_trocar_l1i=getTrocarPose1(0);
    T_arm_l1i=getArmPose1(l1i,theta1i,delta1i,T_d);
    P_arm_l1i=T_arm_l1i(1:3,4);
    P_trocar_l1i=T_trocar_l1i(1:3,4);
    t1=norm(P_trocar_l1i-P_arm_l1i);
    t2=0;
elseif(l1i<1e-4)
    T_trocar_lsi=getTrocarPose1(0);
    T_arm_lsi=getArmPose1(lsi,thetasi,deltasi,T_d);
    P_arm_lsi=T_arm_lsi(1:3,4);
    P_trocar_lsi=T_trocar_lsi(1:3,4);
    t2=norm(P_trocar_lsi-P_arm_lsi);
    t1=0;
else
    T_trocar_lsi=getTrocarPose1(d-lsi);
    T_arm_lsi=getArmPose1(lsi,thetasi,deltasi,T_d);
    P_arm_lsi=T_arm_lsi(1:3,4);
    P_trocar_lsi=T_trocar_lsi(1:3,4);
    t2=norm(P_trocar_lsi-P_arm_lsi);
    T_trocar_l1i=getTrocarPose1(0);
    T_arm_l1i=getArmPose1(l1i,theta1i,delta1i,T_arm_lsi);
    P_arm_l1i=T_arm_l1i(1:3,4);
    P_trocar_l1i=T_trocar_l1i(1:3,4);
    t1=norm(P_trocar_l1i-P_arm_l1i);
end

g=[t1-c t2-c];
h=[];

end