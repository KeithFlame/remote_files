

psi=[pi 40 0 0 0 0];
[phi, lf, theta1, delta1, theta2, delta2]=getPsi(psi);
[r_L1, L1, r_Lr, Lr, r_L2, L2, r_Lg, Lg, r_trocar]=getToolArmStructureParameter;

if(lf<0)
    Ttip=eye(4);
    return;
end
if(lf<Lg)
    
    Ttip=eye(4);
    Ttip(3,4)=lf;
    return;
end
if(lf<sum([L2 Lg]))
    r_curvature_target=(lf-Lg)/theta2;
    dt=2*(r_trocar - max([r_L1 r_Lr r_L2])) + max([r_L1 r_Lr r_L2])-r_L2;
    d=L2+Lg-lf;
    r_curvature_actual_limit=r_L2+(d*d+dt*dt)/2/d;
    
    if(r_curvature_actual_limit>r_curvature_target)
        % 靠边桀了
        theta2_t=atan(dt/d);
        theta2_f=theta2-theta2_t;
        Ttc22e=calcSegib2ie(theta2_t,theta2,d,lf-Lg,L2);
    else
        % 正常计算
        Ttc22e=calcSegib2ie(d/L2*theta2,theta2,lf-Lg,lf-Lg,L2);

    end

    T2e2tip=eye(4);T2e2tip(3,4)=Lg;
    Ttip=Ttc22e*T2e2tip;
end
if(lf<sum([Lr L2 Lg]))
    Ttc22b=eye(4);Ttc22b(3,4)=lf-sum([L2 Lg]);
    d=0;
    Ttc22e=calcSegib2ie(d/L2*theta2,theta2,L2,L2);


end



theta1=2 /180*pi;
L1=100;




t=2;
r=4;
ld=0:100;
block_size=size(ld,2);
Lt=zeros(block_size,1);
for i =1:block_size
    theta1_in = ld(i)/L1*theta1;
    alpha=atan(t/ld(i));
    R=ld(i)/sin(alpha);
    if(alpha<theta1_in)
        Lt(i)=(R+r)*alpha;
    else
        Lt(i)=0;
    end
end
    
figure;plot(ld,Lt);
