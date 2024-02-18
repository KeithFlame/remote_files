function [JvL]=JvLCouple(theta,delta,L1,L0,zeta)
if(L1+L0*zeta<0.0001)
    theta1=0;
    theta0=0;
else
    theta1 = theta*L1/(L1+L0*zeta);
    theta0 = theta*L0*zeta/(L1+L0*zeta);    
end
if(L0<0.00001)
    if(abs(theta1)<1e-7)
        JvL = [0 0 1]';
    else
        JvL = [(1-cos(theta1))/theta1*cos(delta) (1-cos(theta1))/theta1*sin(delta) sin(theta1)/theta1]';
    end
else
    if(abs(theta1)<1e-7)
        JvL=[0 0 1]';
    else
        if(abs(zeta)>0.001)
        JvL = [(-zeta*cos(theta)+(zeta-1)*cos(theta0)-(zeta-1)*theta1*sin(theta0)+1)*cos(delta)/theta ...
               (-zeta*cos(theta)+(zeta-1)*cos(theta0)-(zeta-1)*theta1*sin(theta0)+1)*sin(delta)/theta ...
               (zeta*sin(theta)-(zeta-1)*sin(theta0)-(zeta-1)*theta1*cos(theta0))/theta]';
        else
        JvL =[0 0 1]';
    end
    end

end

end