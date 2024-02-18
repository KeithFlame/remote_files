function [HRB]=PlotRoundBlock(L,W,H,Tbase,color,N)
if(nargin==5)
    N=24;
end
p1=zeros(3,N+2);
for i=0:N+2
    if(i>N/2)
        p1(:,i+1)=[-W/2*sin((i-1)/N*2*pi) W/2*cos((i-1)/N*2*pi) 0]';
        p1(1,i+1)=p1(1,i+1)+L;
    else
        p1(:,i+1)=[-W/2*sin(i/N*2*pi) W/2*cos(i/N*2*pi) 0]';
    end
end
    p1(:,end)=p1(:,1);
    p2=p1-[0 0 H]';
    p1=Tbase(1:3,1:3)*p1+Tbase(1:3,4);
    p2=Tbase(1:3,1:3)*p2+Tbase(1:3,4);
    HRB=[];
    [hup]=patch(p1(1,:),p1(2,:),p1(3,:),color,'linewidth',1,'facealpha',0.2,'edgecolor',color);
    [hlw]=patch(p2(1,:),p2(2,:),p2(3,:),color,'linewidth',1,'facealpha',0.2,'edgecolor',color);
    HRB=[hup hlw];
for i=1:N    
    [h_tmp]=plot3([p1(1,i) p2(1,i)],[p1(2,i) p2(2,i)],[p1(3,i) p2(3,i)],'-k','color',color);
    HRB(end+1)=h_tmp;
end

end