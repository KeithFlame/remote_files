function [p0,r]=spatialCircleFitV2(P)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Project: 3D circle fitting
% Author: jiangjp
%         1034378054@qq.com
%         Wuhan University of Technology
% Date:   25/10/2019
% revised: 6/1/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M=P;
[num dim]=size(M);
L1=ones(num,1);
A=inv(M'*M)*M'*L1;
B=zeros((num-1)*num/2,3);
count=0;
for i=1:num-1
    for j=i+1:num 
        count=count+1;
        B(count,:)=M(j,:)-M(i,:);
    end
end
L2=zeros((num-1)*num/2,1);
count=0;
for i=1:num-1
    for j=i+1:num
        count=count+1;
        L2(count)=(M(j,1)^2+M(j,2)^2+M(j,3)^2-M(i,1)^2-M(i,2)^2-M(i,3)^2)/2;
    end
end
D=zeros(4,4);
D(1:3,1:3)=(B'*B);
D(4,1:3)=A';
D(1:3,4)=A;
L3=[B'*L2;1];
C=inv(D')*(L3);
C=C(1:3);
n=A;
c=C;
radius=0;
for i=1:num
    tmp=M(i,:)-C';
    radius=radius+sqrt(tmp(1)^2+tmp(2)^2+tmp(3)^2);
end
r=radius/num;
p0=C;

h1=plot3(M(:,1),M(:,2),M(:,3),'*');
theta=(0:2*pi/360:(2*pi))';%  theta��   2*pi
% theta=(0:15:180)'*180/pi;
a=cross(n,[1 0 0]);%  n��i��ˣ���ȡa����
if ~any(a)%  ���aΪ����������n��j���
    a=cross(n,[0 1 0]);
end
b=cross(n,a);   % ��ȡb����
a=a/norm(a);     % ��λ��a����
b=b/norm(b);    % ��λ��b����
c1=c(1)*ones(size(theta,1),1);
c2=c(2)*ones(size(theta,1),1);
c3=c(3)*ones(size(theta,1),1);
x=c1+r*a(1)*cos(theta)+r*b(1)*sin(theta);   % Բ�ϸ����x����
y=c2+r*a(2)*cos(theta)+r*b(2)*sin(theta);   % Բ�ϸ����y����
z=c3+r*a(3)*cos(theta)+r*b(3)*sin(theta);   % Բ�ϸ����z����


hold on;
Px=x(1:15:end);Py=y(1:15:end);Pz=z(1:15:end);
% h2=plot3(Px,Py,Pz,'r*');
h2=plot3(x(1:end),y(1:end),z(1:end),'-g');
xlabel('x (mm)')
ylabel('y (mm)')
zlabel('z(mm)')
initV=-89.4440*pi/180;
theta=(initV:2*pi/18:(2*pi+initV))';%  theta��   2*pi
% theta=(0:15:180)'*180/pi;
a=cross(n,[1 0 0]);%  n��i��ˣ���ȡa����
if ~any(a)%  ���aΪ����������n��j���
    a=cross(n,[0 1 0]);
end
b=cross(n,a);   % ��ȡb����
a=a/norm(a);     % ��λ��a����
b=b/norm(b);    % ��λ��b����
c1=c(1)*ones(size(theta,1),1);
c2=c(2)*ones(size(theta,1),1);
c3=c(3)*ones(size(theta,1),1);
x=c1+r*a(1)*cos(theta)+r*b(1)*sin(theta);   % Բ�ϸ����x����
y=c2+r*a(2)*cos(theta)+r*b(2)*sin(theta);   % Բ�ϸ����y����
z=c3+r*a(3)*cos(theta)+r*b(3)*sin(theta);   % Բ�ϸ����z����


hold on;
Px=x(1:15:end);Py=y(1:15:end);Pz=z(1:15:end);
% h2=plot3(Px,Py,Pz,'r*');
h2=plot3(x(1:end),y(1:end),z(1:end),'sb');

d=P-[x,y,z];
ang=zeros(size(d,1),1);
for i = 1 :size(d,1)
    ang(i)=atand(norm(d(i,:))/r);
end
grid on;
axis equal;
view([0 90])
end