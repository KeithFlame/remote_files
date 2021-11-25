function [Jacob]=Jl(u)
%Left Jacobian
if(u==[0 0 0]')
    Jacob=eye(3);
else
    theta = norm(u);
    un=u/theta;
    Jacob=sin(theta)/theta*eye(3)+(1-sin(theta)/theta)*un*un'+(1-cos(theta))/theta*S(un);
end
end