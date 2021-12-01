function T6=plotResult(flag,name)
if nargin==1
    name="";
end
SP=getSP;
arc_radius=SP.trocar.arc_radius;
c=SP.trocar.c;
d=SP.trocar.d;
% inner

T_base=eye(4);T_base(3,4)=-d;
L=SP.dependent_psi.lsi;theta=SP.dependent_psi.thetasi;delta=SP.dependent_psi.deltasi;
[T0,~]=plotSeg(L,theta,delta,T_base);
L=SP.dependent_psi.l1i;theta=SP.dependent_psi.theta1i;delta=SP.dependent_psi.delta1i;
[T1,~]=plotSeg(L,theta,delta,T0);

if(T1(1,4)>c/2)
    T_base(1,4)=c/2-T1(1,4);
    
end
L=SP.dependent_psi.lsi;theta=SP.dependent_psi.thetasi;delta=SP.dependent_psi.deltasi;
[T0,S0]=plotSeg(L,theta,delta,T_base);
L=SP.dependent_psi.l1i;theta=SP.dependent_psi.theta1i;delta=SP.dependent_psi.delta1i;
[T1,S1]=plotSeg(L,theta,delta,T0);
% outer lstem
L=SP.dependent_psi.lso;theta=SP.dependent_psi.thetaso;delta=SP.dependent_psi.deltaso;
[T2,S2]=plotSeg(L,theta,delta,T1);
% l1
L=SP.dependent_psi.l1o;theta=SP.dependent_psi.theta1o;delta=SP.dependent_psi.delta1o;
[T3,S3]=plotSeg(L,theta,delta,T2);
% lr
L=SP.structure.Lr;theta=0;delta=0;
[T4,S4]=plotSeg(L,theta,delta,T3);
% l2
L=SP.structure.L2;theta=SP.psi.theta2;delta=SP.psi.delta2;
[T5,S5]=plotSeg(L,theta,delta,T4);
% lg
L=SP.structure.Lg;theta=0;delta=0;
[T6,S6]=plotSeg(L,theta,delta,T5);


if(flag)
%     figure;
    hold on;grid on;axis equal;view([0 0]);
    title(name);
    xlabel('x');ylabel('y');zlabel('z');
    % plotTrocar;
    line([-0.03, -SP.trocar.c/2],[0, 0],[0, 0],'color','black');
    line([0.03, SP.trocar.c/2],[0, 0],[0, 0],'color','black');
%     line([SP.c/2, SP.c/2],[0, 0],[-SP.d, 0],'color','black');
%     line([-SP.c/2, -SP.c/2],[0, 0],[-SP.d, 0],'color','black');
    theta_1_trocar=SP.trocar.d/arc_radius;
    ub=arc_radius+c/2;
    lb=arc_radius-c/2;
    
    t = (0:-pi/100:-theta_1_trocar);
    x = ub*cos(t)-arc_radius;
    z = ub*sin(t);
    y=t.*0;
    plot3(x,y,z,'b-');
    x = lb*cos(t)-arc_radius;
    z = lb*sin(t);
    y=t.*0;
    plot3(x,y,z,'b-');    


    plot3(S0(1,:), S0(2,:),S0(3,:),'b-');
    plot3(S1(1,:), S1(2,:),S1(3,:),'c-');
    plot3(S2(1,:), S2(2,:),S2(3,:),'r-');
    plot3(S3(1,:), S3(2,:),S3(3,:),'g-');
    plot3(S4(1,:), S4(2,:),S4(3,:),'b-');
    plot3(S5(1,:), S5(2,:),S5(3,:),'c-');
    plot3(S6(1,:), S6(2,:),S6(3,:),'r-');%,LineWidth=2
end
end