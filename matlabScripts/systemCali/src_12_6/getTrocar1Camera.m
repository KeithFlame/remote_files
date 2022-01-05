function [T,z_dev,ang_dev]=getTrocar1Camera(M)

isplot=0;

P=zeros(4,3);
P(1,:)=mean(M([1 3 5],1:3));
P(2,:)=mean(M([2 4 6],1:3));
P(3,:)=mean(M([7 9 11],1:3));
P(4,:)=mean(M([8 10 12],1:3));
if(P(3,1)>P(4,1))
    t=P(3,:);
    P(3,:)=P(4,:);
    P(4,:)=t;
end
% 输入顺序为：Y近端marker坐标、Y远端marker坐标、X近端marker坐标、X远端marker坐标。
n1=(P(2,:)-P(1,:))/norm(P(2,:)-P(1,:));
n2=(P(4,:)-P(3,:))/norm(P(4,:)-P(3,:));
ang_dev=acosd(dot(n1,n2));
% x=[1 1];
% options=
% fmincon('costFunc0',x,[],[],[],[],[],[],[],options);
n3=cross(n2,n1)/norm(cross(n2,n1));
n1=cross(n3,n2)/norm(cross(n3,n2));
R0=[n2' n1' n3'];
% x_axis=R0(1:3,1);y_axis=R0(1:3,2);z_axis=R0(1:3,3);
P0=mean(P)';
Tworld_tem=[R0,P0;[0 0 0 1]];
Prot=zeros(4,3);
for i=1:size(Prot,1)
    temP=inv(Tworld_tem)*[P(i,:) 1]';
    Prot(i,:)=temP(1:3)';
end

y = ( (Prot(1,2)-Prot(2,2))*(Prot(4,2)-Prot(3,2))*Prot(1,1) +...
    (Prot(4,2)-Prot(3,2))*(Prot(2,1)-Prot(1,1))*Prot(1,2)...
    + (Prot(2,2)-Prot(1,2))*(Prot(4,2)-Prot(3,2))*Prot(3,1) + ...
    (Prot(3,1)-Prot(4,1))*(Prot(2,2)-Prot(1,2))*Prot(3,2) ) / ( (Prot(2,1)-Prot(1,1))...
    *(Prot(4,2)-Prot(3,2)) + (Prot(1,2)-Prot(2,2))*(Prot(4,1)-Prot(3,1)) );
x = Prot(1,1) + (Prot(2,1)-Prot(1,1))*(y-Prot(1,2)) / (Prot(2,2)-Prot(1,2));
z=mean(Prot(:,3));
Prot_origin=[x,y,z,1]';
Porigin=Tworld_tem*Prot_origin;
Porigin=Porigin(1:3);
T=[R0,Porigin;[0 0 0 1]];
z_dev=max(Prot(:,3))-min(Prot(:,3));
%% plot

if(isplot)
    figure;hold on;grid on;axis equal;view([-24 -67]);
    plot3(P(:,1),P(:,2),P(:,3),'r*');
    Tcamera_trocar=T;
    x_axis=Tcamera_trocar(1:3,1);y_axis=Tcamera_trocar(1:3,2);z_axis=Tcamera_trocar(1:3,3);
    Porigin=Tcamera_trocar(1:3,4);
    X=Porigin+80*x_axis;
    Y=Porigin+80*y_axis;
    Y_=Porigin-80*y_axis;
    Z=Porigin+80*z_axis;
    plotP=[Porigin';X';Y';Z';Y_'];
    plot3(plotP(1,1),plotP(1,2),plotP(1,3),'o');
    plot3(plotP([1 2],1),plotP([1 2],2),plotP([1 2],3),'r-');
    plot3(plotP([1 3],1),plotP([1 3],2),plotP([1 3],3),'g-');
    % plot3(plotP([1 5],1),plotP([1 5],2),plotP([1 5],3),'g-.');
    plot3(plotP([1 4],1),plotP([1 4],2),plotP([1 4],3),'b-');
    xlabel('X');ylabel('Y');zlabel('Z');
end
end