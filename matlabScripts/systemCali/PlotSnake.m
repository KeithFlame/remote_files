function [T_all]=PlotSnake(psi,seg_lengths,gamma)
%%=======================================%%
% By Yuyang CHEN at 20210630
% psi=[phi,l,theta1,delta1,theta2,delta2]'
% seg_lengths=[L1,Lr,L2,Lg]'
% gamma=0: rigid, gamma=1: flexible
%=========================================%
if(nargin == 2)
    gamma = 0;
end

param=[0.002,1000];
%lengths for l0, l1, lr, l2
length = [max(0,psi(2)-seg_lengths(1)-seg_lengths(2)-seg_lengths(3)) ...
          min(max(0,psi(2)-seg_lengths(2)-seg_lengths(3)),seg_lengths(1)) ...
          min(max(0,psi(2)-seg_lengths(3)),seg_lengths(2)) ...
          min(psi(2),seg_lengths(3))]';
if(length(1)>0)
    [T_tip0]=PlotSeg(psi(4), psi(3)*gamma*length(1)/(gamma*length(1)+length(2)), length(1), expm(S([0 0 0 0 0 psi(1)]')), param);
    [T_tip1]=PlotSeg(psi(4), psi(3)*length(2)/(gamma*length(1)+length(2)), length(2), T_tip0, param);
    [T_tipr]=PlotSeg(0, 0, length(3), T_tip1, param);
    [T_tip2]=PlotSeg(psi(6), psi(5), length(4), T_tipr, param);
    [T_tipg]=PlotSeg(0, 0, seg_lengths(4), T_tip2, param);
elseif(length(2)>0)
    T_tip0=expm(S([0 0 0 0 0 psi(1)]'));
    [T_tip1]=PlotSeg(psi(4), psi(3)*length(2)/(gamma*length(1)+length(2)), length(2), T_tip0, param);
    [T_tipr]=PlotSeg(0, 0, length(3), T_tip1, param);
    [T_tip2]=PlotSeg(psi(6), psi(5), length(4), T_tipr, param);
    [T_tipg]=PlotSeg(0, 0, seg_lengths(4), T_tip2, param);
elseif(length(3)>0)
    T_tip0=expm(S([0 0 0 0 0 psi(1)]'));
    T_tip1 = T_tip0;
    [T_tipr]=PlotSeg(0, 0, length(3), T_tip1, param);
    [T_tip2]=PlotSeg(psi(6), psi(5), length(4), T_tipr, param);
    [T_tipg]=PlotSeg(0, 0, seg_lengths(4), T_tip2, param);
else
    T_tip0=expm(S([0 0 0 0 0 psi(1)]'));
    T_tip1=T_tip0;
    T_tipr=T_tip1;
    [T_tip2]=PlotSeg(psi(6), psi(5), length(4), T_tipr, param);
    [T_tipg]=PlotSeg(0, 0, seg_lengths(4), T_tip2, param);
end


T_all.T_tip0=T_tip0;
T_all.T_tip1=T_tip1;
T_all.T_tipr=T_tipr;
T_all.T_tip2=T_tip2;
T_all.T_tipg=T_tipg;
end