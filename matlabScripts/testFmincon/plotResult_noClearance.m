function T=plotResult_noClearance(flag,d,name)
if nargin==1
    name="";
    d=0.01;
end
if nargin==2
    name="";
end
SP=setInitVal;
% figure;
d_before=d;
psi=[SP.phi,d_before+SP.l-SP.Lr-SP.L2,SP.theta1,SP.delta1,SP.theta2,SP.delta2];
LS=zeros(10,1);
LS(1)=SP.L1;LS(2)=SP.Lr;LS(3)=SP.L2;LS(4)=SP.Lg;LS(5)=SP.zeta;
[T,~,~,~,~,~,S1,S2,S3,S4,S5]=fromPsi2Config(psi,LS);

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
    line([-0.03, -SP.c/2],[0, 0],[0, 0],'color','black');
    line([0.03, SP.c/2],[0, 0],[0, 0],'color','black');
    line([SP.c/2, SP.c/2],[0, 0],[-SP.d, 0],'color','black');
    line([-SP.c/2, -SP.c/2],[0, 0],[-SP.d, 0],'color','black');
    plot3(S1(1,:), S1(2,:),S1(3,:),'r');
    plot3(S2(1,:), S2(2,:),S2(3,:),'g');
    plot3(S3(1,:), S3(2,:),S3(3,:),'b');
    plot3(S4(1,:), S4(2,:),S4(3,:),'c');
    plot3(S5(1,:), S5(2,:),S5(3,:),'r');
end
