function d=getd
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.22.2021
% Ver. 1.0
% input1: lagrange multipliers
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
du=getDeltaU;
Xk=getXk;
xkT=getxkT;
qk=getQk;

d=0.5*du*(Xk')*Xk*du+xkT*Xk*du+qk;

end