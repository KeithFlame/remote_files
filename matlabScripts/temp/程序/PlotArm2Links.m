function [hAll]=PlotArm2Links(arm,config_i)
color1=[0.6 0.2 0.2];
color2=[0.0 0.4 0.3];
color3=[0.7 0.6 0.1];
if (arm.id==2 || arm.id==3)
    clr1 = color1;
    clr2 = color2;
else
    clr1 = color2;
    clr2 = color3;
end
[t]=forwardArm_2Links(arm,config_i);
[hArm1]=PlotRoundBlock(arm.L1,arm.W1,arm.H1,t.T1_b,clr1);
[hArm2]=PlotRoundBlock(arm.L2,arm.W2,arm.H2,t.T2_b*[eye(3) [0 0 -arm.H1]';0 0 0 1],clr2);
[hArm2_]=PlotRoundBlock(0,arm.W2,arm.H2_,t.T2_b*[eye(3) [arm.L2 0 -arm.H1-arm.H2]';0 0 0 1],clr2,24);
hAll=[hArm1 hArm2 hArm2_];
end
