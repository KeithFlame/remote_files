function Pr=plotCoord(T,length)
if nargin==1
    length=1;
end
    arrowLen=length*0.01;
    lineWidth=length;
    R=T(1:3,1:3);
    P=T(1:3,4);
    px = R*[arrowLen 0 0]'+P;
    py = R*[0 arrowLen 0]'+P;
    pz = R*[0 0 arrowLen]'+P;
%     line([P(1) px(1)],[P(2) px(2)],[P(3) px(3)],'Color','r','LineStyle','-','LineWidth',lineWidth);
%     line([P(1) py(1)],[P(2) py(2)],[P(3) py(3)], 'Color','g','LineStyle','-','LineWidth',lineWidth);
    line([P(1) pz(1)],[P(2) pz(2)],[P(3) pz(3)], 'Color','b','LineStyle','-','LineWidth',lineWidth);
    plot3(P(1),P(2),P(3),'c*');
    Pr=[P px P py P pz];
end