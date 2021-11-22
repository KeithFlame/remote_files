function qk=getQk(Pi,Pi1,Pi_dot,ci)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.18.2021
% Ver. 1.0
% input1: posistion
% input2: clearance
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%% projection matrix
Pis=eye(3)-Pi_dot*Pi_dot';
Sip=Pi-Pi1;
xi=Pis*Sip;
qk=1/2*(xi'*xi-ci^2);

end
