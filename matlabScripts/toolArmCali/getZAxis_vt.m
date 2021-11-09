
function [z ,cter, P0]=getZAxis_vt(data_Z)


P=[
    data_Z(:,1:3);
    data_Z(:,9:11) 

    ];
% P_block=[ ];
P=sortrows(P,1);
x=P(:,1);y=P(:,2);z=P(:,3);
xyz0(1)=mean(x);
xyz0(2)=mean(y);
xyz0(3)=mean(z);
centeredLine=bsxfun(@minus,[x y z],xyz0);
[U,S,V]=svd(centeredLine);
direction=V(:,1);
P0=xyz0;
dis=zeros(size(P,1),1);
for i =1:size(P,1)
    dis(i) = norm(cross(direction,P(i,:)-P0))/norm(direction);
end
t=max(dis);
x_hat=P(1,1):P(end,1);
y_hat=(x_hat-P0(1)).*direction(2)/direction(1)+P0(2);
z_hat=(x_hat-P0(1)).*direction(3)/direction(1)+P0(3);
% figure;hold on;
% plot3(x,y,z,'*');
% plot3(x_hat,y_hat,z_hat,'-');
% plot3(P_block(:,1),P_block(:,2),P_block(:,3),'r*');
% axis equal;%axis([-40 90 -20 20 180 210])
err=1-max(dis)/norm(P(1,:)-P(end,:));
% xlabel("x");ylabel("y");zlabel("z");
cter=err;
z=direction;




