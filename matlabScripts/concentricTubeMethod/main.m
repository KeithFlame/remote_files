% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.18.2021
% Ver. 1.0
% main
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
[u,P,R,ds]=getForwardKinematics();
cter=1;
alpha = 1.0;
while(cter>1e-6)
    delta_u=getInnerIterationRes(u,P,R,ds);
    u=u+alpha*delta_u;
    [u,P,R,ds]=getForwardKinematics(u);
    cter=norm(u);
end
