function [R]=Expm(u)
%simplified calculation for exponetial map (u in R3)
theta=norm(u);
if(theta == 0)
    R=eye(3);
else
    un=u/theta;
    R=cos(theta)*eye(3)+(1-cos(theta))*un*un'+sin(theta)*S(un);
end
end
