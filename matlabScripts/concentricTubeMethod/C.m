function Cu=C(u)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.18.2021
% Ver. 1.0
% input1: curvature*arcLength
% output: C
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
u=reshape(u,[3 1]);
alpha=getAlpha(u);
beta=getBeta(u);
ez=[0 0 1]';
Rez=getSkewMatrix(ez);
Ru=getSkewMatrix(u);
nu=norm(u);
Cu=-(1-beta)*Rez/2 + (1-alpha)/nu/nu*(Rez*Ru +Ru*Rez)+(alpha-beta)/nu/nu*(u'*ez)*Ru...
    +1/nu/nu*(0.5*beta-3*(1-alpha)/nu/nu)*(u'*ez)*Ru*Ru;

end