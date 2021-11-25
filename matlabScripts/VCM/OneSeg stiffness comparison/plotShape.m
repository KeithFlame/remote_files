function [hShape]=plotShape(p,Rs,N,MP,color)
%-----plot variable curvature robot shape from continuum robots
% p(N1+N2 by 3)-central backbone positions, Rs(N1+N2 by 9)-central backbone
%orientations;
% N=[N1,N2]-number of points;
% MP-Mechanical properties;
%---------info----------%
% By Yuyang Chen
% ver p1.2
% date 20200606
%------------------------------------------------------------------------%
if(nargin == 4)
    %color=[0.8 0.4 0];
    color=[0 0 0];
end


rho1=[1 0 0]'*MP.rho;rho2=[0 1 0]'*MP.rho;rho3=[-1 0 0]'*MP.rho;rho4=[0 -1 0]'*MP.rho;
rho5=[sqrt(2)/2*1 sqrt(2)/2*1 0]'*MP.rho;rho6=[-sqrt(2)/2*1 sqrt(2)/2*1 0]'*MP.rho;
rho7=[-sqrt(2)/2*1 -sqrt(2)/2*1 0]'*MP.rho;rho8=[sqrt(2)/2*1 -sqrt(2)/2*1 0]'*MP.rho;

N1=N(1);N2=N(2);
p1=zeros(N1,3);p2=zeros(N1,3);p3=zeros(N1,3);p4=zeros(N1,3);
p5=zeros(N1+N2,3);p6=zeros(N1+N2,3);p7=zeros(N1+N2,3);p8=zeros(N1+N2,3);

for i=1:N1
   p1(i,:)=p(i,:)+rho1'*[Rs(i,1:3);Rs(i,4:6);Rs(i,7:9)];
   p2(i,:)=p(i,:)+rho2'*[Rs(i,1:3);Rs(i,4:6);Rs(i,7:9)];
   p3(i,:)=p(i,:)+rho3'*[Rs(i,1:3);Rs(i,4:6);Rs(i,7:9)];
   p4(i,:)=p(i,:)+rho4'*[Rs(i,1:3);Rs(i,4:6);Rs(i,7:9)];
   
   p5(i,:)=p(i,:)+rho5'*[Rs(i,1:3);Rs(i,4:6);Rs(i,7:9)];
   p6(i,:)=p(i,:)+rho6'*[Rs(i,1:3);Rs(i,4:6);Rs(i,7:9)];
   p7(i,:)=p(i,:)+rho7'*[Rs(i,1:3);Rs(i,4:6);Rs(i,7:9)];
   p8(i,:)=p(i,:)+rho8'*[Rs(i,1:3);Rs(i,4:6);Rs(i,7:9)]; 
end
for i=N1+1:N1+N2
   p5(i,:)=p(i,:)+rho5'*[Rs(i,1:3);Rs(i,4:6);Rs(i,7:9)];
   p6(i,:)=p(i,:)+rho6'*[Rs(i,1:3);Rs(i,4:6);Rs(i,7:9)];
   p7(i,:)=p(i,:)+rho7'*[Rs(i,1:3);Rs(i,4:6);Rs(i,7:9)];
   p8(i,:)=p(i,:)+rho8'*[Rs(i,1:3);Rs(i,4:6);Rs(i,7:9)]; 
end
figure(1);hold on;
hc=plot3(p(1:end-1,1),p(1:end-1,2),p(1:end-1,3),'--','LineWidth',1,'Color',color);
%hc2=plot3(p(end-1:end,1),p(end-1:end,2),p(end-1:end,3),'-','LineWidth',5,'Color',color);
hg=drawGripper(p(end-1,:)',[Rs(end,1:3);Rs(end,4:6);Rs(end,7:9)]',MP.rho*0.9,MP.Lg,color);
h1=plot3(p1(1:end-1,1),p1(1:end-1,2),p1(1:end-1,3),'-','LineWidth',1,'Color',color);
h2=plot3(p2(1:end-1,1),p2(1:end-1,2),p2(1:end-1,3),'-','LineWidth',1,'Color',color);
h3=plot3(p3(1:end-1,1),p3(1:end-1,2),p3(1:end-1,3),'-','LineWidth',1,'Color',color);
h4=plot3(p4(1:end-1,1),p4(1:end-1,2),p4(1:end-1,3),'-','LineWidth',1,'Color',color);
hr=drawRigid(p(N1-1,:)',[Rs(N1-1,1:3);Rs(N1-1,4:6);Rs(N1-1,7:9)]',MP.rho*1.1,MP.Lr,color);
if(N(2)~=1)
h5=plot3(p5(1:end-1,1),p5(1:end-1,2),p5(1:end-1,3),'-','LineWidth',1,'Color',color);
h6=plot3(p6(1:end-1,1),p6(1:end-1,2),p6(1:end-1,3),'-','LineWidth',1,'Color',color);
h7=plot3(p7(1:end-1,1),p7(1:end-1,2),p7(1:end-1,3),'-','LineWidth',1,'Color',color);
h8=plot3(p8(1:end-1,1),p8(1:end-1,2),p8(1:end-1,3),'-','LineWidth',1,'Color',color);
else
    h5=[];
    h6=[];
    h7=[];
    h8=[];
end

hD1=drawCircle(p(N1-1,:),[Rs(N1-1,1:3);Rs(N1-1,4:6);Rs(N1-1,7:9)]',MP.rho,color);
if(N2~=0)
hD2=drawCircle(p(N1,:),[Rs(N1,1:3);Rs(N1,4:6);Rs(N1,7:9)]',MP.rho,color);
hD3=drawCircle(p(N1+N2-1,:),[Rs(N1+N2-1,1:3);Rs(N1+N2-1,4:6);Rs(N1+N2-1,7:9)]',MP.rho,color);
else
    hD2=[];
    hD3=[];
end
%hD4=drawCircle(p(N1+N2,:),[Rs(N1+N2,1:3);Rs(N1+N2,4:6);Rs(N1+N2,7:9)]',MP.rho,color);
hShape=[hc hg h1 h2 h3 h4 hr h5 h6 h7 h8 hD1 hD2 hD3]';
grid on;axis equal;
axis([-0.05 0.05 -0.05 0.05 0 0.1]);

end