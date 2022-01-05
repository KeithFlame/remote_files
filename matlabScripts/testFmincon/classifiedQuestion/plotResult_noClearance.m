function T=plotResult_noClearance(flag,flag_k,d,name)
if nargin==2
    name="";
    d=0.01;
end
if nargin==3
    name="";
end
if(flag_k==1)
    SP=getSP_1;
elseif(flag_k==2)
    SP=getSP_2;
elseif(flag_k==3)
    SP=getSP_3;
else 
    SP=getSP_1;
end
% figure;
d_before=d;
psi=[SP.psi.phi,d_before+SP.psi.l-SP.structure.Lr-SP.structure.L2,SP.psi.theta1,SP.psi.delta1,SP.psi.theta2,SP.psi.delta2];
LS=zeros(10,1);
LS(1)=SP.structure.L1;LS(2)=SP.structure.Lr;LS(3)=SP.structure.L2;LS(4)=SP.structure.Lg;LS(5)=SP.structure.zeta;
[T,~,~,~,~,~,S1,S2,S3,S4,S5]=fromPsi2Config(psi,LS);
d_before=d_before+0;
S1=S1-[0 0 d_before]';
S2=S2-[0 0 d_before]';
S3=S3-[0 0 d_before]';
S4=S4-[0 0 d_before]';
S5=S5-[0 0 d_before]';
T(3,4)=T(3,4)-d_before;

if(flag)
    hold on;grid on;axis equal;view([0 0]);
    title(name);
    xlabel('x');ylabel('y');zlabel('z');
    % plotTrocar;
    line([-0.03, -SP.trocar.c/2],[0, 0],[0, 0],'color','black');
    line([0.03, SP.trocar.c/2],[0, 0],[0, 0],'color','black');
    line([SP.trocar.c/2, SP.trocar.c/2],[0, 0],[-SP.trocar.d, 0],'color','black');
    line([-SP.trocar.c/2, -SP.trocar.c/2],[0, 0],[-SP.trocar.d, 0],'color','black');
    plot3(S1(1,:), S1(2,:),S1(3,:),'r');
    plot3(S2(1,:), S2(2,:),S2(3,:),'g');
    plot3(S3(1,:), S3(2,:),S3(3,:),'b');
    plot3(S4(1,:), S4(2,:),S4(3,:),'c');
    plot3(S5(1,:), S5(2,:),S5(3,:),'r');
end
