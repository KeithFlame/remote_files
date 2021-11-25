function [hR]=drawRigid(p,R,rho,Lr,color)
if(nargin==4)
    color=[0 0 0];
end
if(Lr == 0)
    hR=[];
    return;
end
N=16;%even
[x,y,z]=cylinder(rho,N);
p0=zeros(3,2*N);
for i=1:N
    if(mod(i,2)==1)
    p0(:,2*i-1)=[x(1,i) y(1,i) Lr*z(1,i)]';
    p0(:,2*i)=[x(2,i) y(2,i) Lr*z(2,i)]';
    else
    p0(:,2*i-1)=[x(2,i) y(2,i) Lr*z(2,i)]';
    p0(:,2*i)=[x(1,i) y(1,i) Lr*z(1,i)]';
    end
end
p0(:,2*N+1:2*N+2)=p0(:,1:2);
p1=p+R*p0;
hR=[];
for i=1:2:2*N
    h1=patch(p1(1,i:i+3),p1(2,i:i+3),p1(3,i:i+3),color,'FaceColor',color,'FaceAlpha',0.4,'EdgeAlpha',0,'EdgeColor',color,'LineWidth',1);
    hR=[hR h1];
end

end