function [y,cter]=getYAxis(data_Y)
global Pvy;
%D0921
m1=data_Y(:,1:3);
m2=data_Y(:,9:11);
Pvy0=zeros(size(m2,1),3);
Pvy2=zeros(size(m2,1),1);
for i =1:size(m2,1)
    Pvy0(i,:)=m2(i,:)-m1(i,:);
    if(m2(i,2)<m1(i,2))
        Pvy0(i,:)=-m2(i,:)+m1(i,:);
    end
    Pvy2(i)=norm(Pvy0(i,:));
end




Pvy0(:,3)=Pvy0(:,3);
Pvy=Pvy0;
% figure;hold on;
% plot3(Pvy(:,1),Pvy(:,2),Pvy(:,3),'*');
% axis equal;
% axis([-1 1 24.5 26.5 -1 1]);
% xlabel("x");ylabel("y");zlabel("z");



options = optimoptions('fmincon','Algorithm','interior-point'); % 
options.StepTolerance=1e-20;
[x1,y0,exitflag]=fmincon('costFuncToGetY',[0 1 0],[],[],[],[],[],[],[],options);
% plot3(x1(1),x1(2),x1(3),'r*');
[x,y,z] = sphere(20);
y0=y0/1000;
x = y0*x; 
y = y0*y;
z = y0*z;
x = x+x1(1);
y = y+x1(2);
z = z+x1(3);
% mesh(x,y,z);
% s = surf(x,y,z,'FaceAlpha',0.3);
% s.EdgeColor = 'none';
y=x1'/norm(x1);
cter=y0;
end