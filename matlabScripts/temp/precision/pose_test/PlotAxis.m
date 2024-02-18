function PlotAxis(scale,Te)
%-----plot frames
% ver p1.0
% By Yuyang Chen
% Date 20200606
%-----------------------%
    a=linspace(0,scale,2);
    b=zeros(1,2);
    Re=Te(1:3,1:3);Pe=Te(1:3,4);
    X=[a;b;b];Y=[b;a;b];Z=[b;b;a];
    X=Re*X+[Pe Pe];
    Y=Re*Y+[Pe Pe];
    Z=Re*Z+[Pe Pe];
    hold on;
%     hX=[];hY=[];hZ=[];hO=[];
    plot3(X(1,:),X(2,:),X(3,:),'-r','LineWidth',1);
    plot3(Y(1,:),Y(2,:),Y(3,:),'-g','LineWidth',1);
    plot3(Z(1,:),Z(2,:),Z(3,:),'-b','LineWidth',1);%,'Color',[0.8 0.6 0.8]);
    plot3(Pe(1),Pe(2),Pe(3),'o','MarkerSize',4,'color','c');
%     hAx=[hX hY hZ hO];
end