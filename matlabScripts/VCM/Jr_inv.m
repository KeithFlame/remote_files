function [Jacob]=Jr_inv(u)
%Left Jacobian
if(u==[0 0 0]')
    Jacob=eye(3);
else
    theta = norm(u);
    un=u/theta;
    Jacob=theta/2*cot(theta/2)*eye(3)+(1-theta/2*cot(theta/2))*un*un'+theta/2*S(un);
end
end