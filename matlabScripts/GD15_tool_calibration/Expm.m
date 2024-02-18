function [T]=Expm(u)
%simplified calculation for exponetial map (u in R3)
if(length(u)==3)
    theta=norm(u);
    if(theta == 0)
        R=eye(3);
    else
        un=u/theta;
        R=cos(theta)*eye(3)+(1-cos(theta))*un*un'+sin(theta)*S(un);
    end
    T=R;
elseif(length(u)==6)
    theta=norm(u(4:6));
    if(theta == 0)
        R=eye(3);
    else
        un=u(4:6)/theta;
        R=cos(theta)*eye(3)+(1-cos(theta))*un*un'+sin(theta)*S(un);
    end
    T=[R u(1:3);0 0 0 1];
else
    T=eye(4);
    disp('erroneous size of u!');
end
end
