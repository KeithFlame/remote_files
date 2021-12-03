l=100e-3;
theta1=pi/2;
SP.psi.theta1=theta1;
SP.psi.l=l;
SP.psi.phi=4.1108;
SP.psi.delta1=0;
SP.psi.theta2=pi/2-theta1;
SP.psi.delta2=0;

SP=setInitValV2(SP);
Lr=SP.structure.Lr;
L2=SP.structure.L2;
l=SP.psi.l;
L1=SP.structure.L1;
zeta=SP.structure.zeta;
c=SP.trocar.c;
OP=setOP;
pre_d=10e-3;
r_zero_pose=getZeroPose(theta1);
if(r_zero_pose>c/2)

    if(l>L1+L2+Lr)
        [xh,yh,exitflag_1]=fmincon('costFunc_1',OP.x1,[],[],[],[],OP.xhmin1,OP.xhmax1,'nonlcon_1',OP.options);
    elseif(l<L2+Lr+L1-pre_d)
        [xh,yh,exitflag_1]=fmincon('costFunc_2',OP.x2,[],[],[],[],OP.xhmin2,OP.xhmax2,'nonlcon_2',OP.options);
    else
        [xh,yh,exitflag_1]=fmincon('costFunc_3',OP.x3,[],[],[],[],OP.xhmin3,OP.xhmax3,'nonlcon_3',OP.options);
    end

    Tend=getEndV2();
else
    Tend=getEnd();
end


