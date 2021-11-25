function [hAx]=plotAxis(scale,Te)
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
    hX=plot3(X(1,:),X(2,:),X(3,:),'-r','LineWidth',3);
    hY=plot3(Y(1,:),Y(2,:),Y(3,:),'-b','LineWidth',3);
    hZ=plot3(Z(1,:),Z(2,:),Z(3,:),'-','color',[0 0.65 0],'LineWidth',3);%,'Color',[0.8 0.6 0.8]);
    hAx=[hX hY hZ];
end