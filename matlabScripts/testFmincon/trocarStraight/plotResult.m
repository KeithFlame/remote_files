function T6=plotResult(flag,name)
if nargin==1
    name="";
end
SP=getSPV1;

% inner

thetasi=SP.dependent_psi.thetasi;
theta1i=SP.dependent_psi.theta1i;
dsi=SP.dependent_psi.dsi;
d1i=SP.dependent_psi.d1i;
thetaso=SP.dependent_psi.thetaso;
theta1o=SP.dependent_psi.theta1o;
lso=SP.dependent_psi.lso;
l1o=SP.dependent_psi.l1o;
[Lsi,~]=getLU1Trocar(thetasi,dsi);
[Lso,~]=getLUNot1Trocar(thetaso,lso);
[L1i,~]=getLU1Trocar(theta1i,d1i);
[L1o,~]=getLUNot1Trocar(theta1o,l1o);




T_base=eye(4);T_base(3,4)=-SP.trocar.d;
phi=SP.psi.phi;
T_base(1:3,1:3)=[cos(phi) -sin(phi) 0;sin(phi) cos(phi) 0;0 0 1];
L=Lsi;theta=thetasi;delta=SP.dependent_psi.deltasi;
[T0,~]=plotSeg(L,theta,delta,T_base);
L=L1i;theta=theta1i;delta=SP.dependent_psi.delta1i;
[T1,~]=plotSeg(L,theta,delta,T0);


tem_norm=norm(T1(1:2,4));
if(tem_norm>SP.trocar.c/2)
    T_base(1:2,4)=(SP.trocar.c/2-tem_norm)/tem_norm*T1(1:2,4);
end
L=Lsi;theta=thetasi;delta=SP.dependent_psi.deltasi;
[T0,S0]=plotSeg(L,theta,delta,T_base);
L=L1i;theta=theta1i;delta=SP.dependent_psi.delta1i;
[T1,S1]=plotSeg(L,theta,delta,T0);
% outer lstem
L=Lso;theta=thetaso;delta=SP.dependent_psi.deltaso;
[T2,S2]=plotSeg(L,theta,delta,T1);
% l1
L=L1o;theta=theta1o;delta=SP.dependent_psi.delta1o;
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


d_before=0;
S1=S1-[0 0 d_before]';
S2=S2-[0 0 d_before]';
S3=S3-[0 0 d_before]';
S4=S4-[0 0 d_before]';
S5=S5-[0 0 d_before]';
T6(3,4)=T6(3,4)-d_before;

if(flag)
%     figure;
    hold on;grid on;axis equal;view([0 0]);
    title(name);
    ax = gca;
    ax.FontName = 'Times New Roman';
    xlabel('{\itx} (m)');ylabel('{\ity} (m)');zlabel('{\itz} (m)');
%     % plotTrocar;
%     line([-0.03, -SP.trocar.c/2],[0, 0],[0, 0],'color','black');
%     line([0.03, SP.trocar.c/2],[0, 0],[0, 0],'color','black');
%     line([SP.trocar.c/2, SP.trocar.c/2],[0, 0],[-SP.trocar.d, 0],'color','black');
%     line([-SP.trocar.c/2, -SP.trocar.c/2],[0, 0],[-SP.trocar.d, 0],'color','black');
%     line([0, 0],[SP.trocar.c/2, SP.trocar.c/2],[-SP.trocar.d, 0],'color','black');
%     line([0, 0],[-SP.trocar.c/2, -SP.trocar.c/2],[-SP.trocar.d, 0],'color','black');
%     r=SP.trocar.c/2;
%     rectangle('Position',[-r,-r,2*r,2*r],'Curvature',[1,1],'linewidth',1,'edgecolor','black');
%     theta_circle=0:0.1:2*pi;
%     Circle_x=r*cos(theta_circle);
%     Circle_y=r*sin(theta_circle);
%     Circle_z=-SP.trocar.d+0.*theta_circle;
%     plot3(Circle_x, Circle_y,Circle_z,'black');

    values0 = spcrv([[S0(1,1) S0(1,:) S0(1,end)];[S0(2,1) S0(2,:) S0(2,end)]; ...
        [S0(3,1) S0(3,:) S0(3,end)]],3);
    values1 = spcrv([[S1(1,1) S1(1,:) S1(1,end)];[S1(2,1) S1(2,:) S1(2,end)]; ...
        [S1(3,1) S1(3,:) S1(3,end)]],3);
    values2 = spcrv([[S2(1,1) S2(1,:) S2(1,end)];[S2(2,1) S2(2,:) S2(2,end)]; ...
        [S2(3,1) S2(3,:) S2(3,end)]],3);
    values3 = spcrv([[S3(1,1) S3(1,:) S3(1,end)];[S3(2,1) S3(2,:) S3(2,end)]; ...
        [S3(3,1) S3(3,:) S3(3,end)]],3);
    values4 = spcrv([[S4(1,1) S4(1,:) S4(1,end)];[S4(2,1) S4(2,:) S4(2,end)]; ...
        [S4(3,1) S4(3,:) S4(3,end)]],3);
    values5 = spcrv([[S5(1,1) S5(1,:) S5(1,end)];[S5(2,1) S5(2,:) S5(2,end)]; ...
        [S5(3,1) S5(3,:) S5(3,end)]],3);
    values6 = spcrv([[S6(1,1) S6(1,:) S6(1,end)];[S6(2,1) S6(2,:) S6(2,end)]; ...
        [S6(3,1) S6(3,:) S6(3,end)]],3);
%     plot3(values0(1,:),values0(2,:),values0(3,:))




    plot3(values0(1,:), values0(2,:),values0(3,:),'b-');
    plot3(values1(1,:), values1(2,:),values1(3,:),'c-');
    plot3(values2(1,:), values2(2,:),values2(3,:),'r-');
    plot3(values3(1,:), values3(2,:),values3(3,:),'g-');
    plot3(values4(1,:), values4(2,:),values4(3,:),'b-');
    plot3(values5(1,:), values5(2,:),values5(3,:),'c-');
    plot3(values6(1,:), values6(2,:),values6(3,:),'black');%,LineWidth=2


    [x,y,z]=cylinder(SP.trocar.c/2);  
    z(2,:)=-SP.trocar.d;      
    s2=surf(x,y,z,'FaceColor',[0.7,0.7,0.7],'FaceAlpha',0.5);       
%     s2.AlphaData = 0.01;
    s2.EdgeColor = 'none';

end
end