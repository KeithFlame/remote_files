function [Jw]=JwSegCouple(theta,delta,L1,L0,zeta)
if(L1+L0*zeta<0.0001)
    Jw = zeros(3,2);
else
    theta1 = theta*L1/(L1+L0*zeta);
    theta0 = theta*L0*zeta/(L1+L0*zeta);
    Jw = [-sin(delta) -sin(theta1+theta0)*cos(delta); ...
           cos(delta) -sin(theta1+theta0)*sin(delta); ...
           0 1-cos(theta1+theta0)];
end
end