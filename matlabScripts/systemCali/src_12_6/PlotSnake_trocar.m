function [T_all]=PlotSnake_trocar(psi_in,seg_lengths,gamma,theta_trocar,len_trocar,bend_in_trocar,offset_zero)
psi= psi_in;
psi(2)=psi(2)+bend_in_trocar+offset_zero;
param=[0.004,1000];
%lengths for l0, l1, lr, l2
length = [max(0,psi(2)-seg_lengths(1)-seg_lengths(2)-seg_lengths(3)) ...
          min(max(0,psi(2)-seg_lengths(2)-seg_lengths(3)),seg_lengths(1)) ...
          min(max(0,psi(2)-seg_lengths(3)),seg_lengths(2)) ...
          min(psi(2),seg_lengths(3))]';

if(length(1)<len_trocar)
    length(1) = 0;
    length(2) = length(2)-(len_trocar-length(1));
else
    length(1)=length(1)-len_trocar;
end
[T_tip_trocar]=PlotSeg(psi(4),theta_trocar,len_trocar,expm(S([0 0 -bend_in_trocar 0 0 psi(1)]')), param);
psi(3)=psi(3)-theta_trocar;
if(length(1)>0)
    [T_tip0]=PlotSeg(psi(4), psi(3)*gamma*length(1)/(gamma*length(1)+length(2)), length(1), T_tip_trocar, param);
    [T_tip1]=PlotSeg(psi(4), psi(3)*length(2)/(gamma*length(1)+length(2)), length(2), T_tip0, param);
    [T_tipr]=PlotSeg(0, 0, length(3), T_tip1, param);
    [T_tip2]=PlotSeg(psi(6), psi(5), length(4), T_tipr, param);
    [T_tipg]=PlotSeg(0, 0, seg_lengths(4), T_tip2, param);
elseif(length(2)>0)
    T_tip0=T_tip_trocar;
    [T_tip1]=PlotSeg(psi(4), psi(3)*length(2)/(gamma*length(1)+length(2)), length(2), T_tip0, param);
    [T_tipr]=PlotSeg(0, 0, length(3), T_tip1, param);
    [T_tip2]=PlotSeg(psi(6), psi(5), length(4), T_tipr, param);
    [T_tipg]=PlotSeg(0, 0, seg_lengths(4), T_tip2, param);
elseif(length(3)>0)
    T_tip0=T_tip_trocar;
    T_tip1 = T_tip0;
    [T_tipr]=PlotSeg(0, 0, length(3), T_tip1, param);
    [T_tip2]=PlotSeg(psi(6), psi(5), length(4), T_tipr, param);
    [T_tipg]=PlotSeg(0, 0, seg_lengths(4), T_tip2, param);
else
    T_tip0=T_tip_trocar;
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