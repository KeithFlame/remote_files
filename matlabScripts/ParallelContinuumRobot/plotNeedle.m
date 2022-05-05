function plotNeedle(T)
%   this is function to plot needle
%   input 1 is the pose of the needle
%
%   Author Keith W.
%   Ver. 1.0
%   Date 04.29.2022

SP = getStructurePara_keith;
needle=SP.needle;
theta = 0:4:needle.l*360;
offset = 180;
block_size = size(theta,2);
x=cosd(theta+offset)*needle.r;
y=sind(theta+offset)*needle.r;
z=zeros(1,block_size);
p=[x;y;z];
R=T(1:3,1:3);P=T(1:3,4);
p=R*p+P;
hold on;
plot3(p(1,:),p(2,:),p(3,:),'m-',lineWidth=needle.d*3);
plotCoord(T,3);
end


%% 画坐标系
function Pr=plotCoord(T,length)
if nargin==1
    length=1;
end
    arrowLen=length*3;
    lineWidth=length;
    R=T(1:3,1:3);
    P=T(1:3,4);
    px = R*[arrowLen 0 0]'+P;
    py = R*[0 arrowLen 0]'+P;
    pz = R*[0 0 arrowLen]'+P;
    line([P(1) px(1)],[P(2) px(2)],[P(3) px(3)],'Color','r','LineStyle','-','LineWidth',lineWidth);
    line([P(1) py(1)],[P(2) py(2)],[P(3) py(3)], 'Color','g','LineStyle','-','LineWidth',lineWidth);
    line([P(1) pz(1)],[P(2) pz(2)],[P(3) pz(3)], 'Color','b','LineStyle','-','LineWidth',lineWidth);
%     plot3(P(1),P(2),P(3),'c*');
    Pr=[P px P py P pz];
end

