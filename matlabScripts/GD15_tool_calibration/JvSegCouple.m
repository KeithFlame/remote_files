function [Jv] = JvSegCouple(theta,delta,L1,L0,zeta)
if(L1+L0*zeta < 0.0001)
    Jv = zeros(3,2);
else
    theta1 = theta*L1/(L1+L0*zeta);
    theta0 = theta*L0*zeta/(L1+L0*zeta);
    if(zeta>0.001)
        if(abs(theta1)<1e-7)
            Jv = [L1/2*(zeta*L0^2/L1^2 + 2*zeta*L0/L1 + 1)*cos(delta)*L1/(L1+zeta*L0) 0; ...
                  L1/2*(zeta*L0^2/L1^2 + 2*zeta*L0/L1 + 1)*sin(delta)*L1/(L1+L0*zeta) 0; ...
                  0 0];
        else
            B1 = L1/theta1*(1/zeta + cos(theta0)*(1-1/zeta - cos(theta1)) + sin(theta0)*sin(theta1));
            B2 = L1/theta1*(cos(theta0)*sin(theta1)- sin(theta0)*(1-1/zeta-cos(theta1)));
            C1 = L1/theta1*sin(theta0)*cos(theta1)-zeta*L0/theta1*sin(theta0)*(1-1/zeta-cos(theta1)) + (L1+L0*zeta)/theta1*cos(theta0)*sin(theta1);
            C2 = L1/theta1*cos(theta0)*cos(theta1)-zeta*L0/theta1*cos(theta0)*(1-1/zeta-cos(theta1)) - (L1+L0*zeta)/theta1*sin(theta0)*sin(theta1);
            Jv = [(-B1*cos(delta)/theta1+C1*cos(delta))*L1/(L1+L0*zeta) -B1*sin(delta); ...
                  (-B1*sin(delta)/theta1+C1*sin(delta))*L1/(L1+L0*zeta)  B1*cos(delta); ...
                  (-B2/theta1+C2)*L1/(L1+L0*zeta) 0];
        end
    else
        if(abs(theta1)<1e-7)
            Jv = [L1*cos(delta)/2 0; L1*sin(delta)/2 0; 0 0];
        else
            B1 = L1/theta1*(1-cos(theta1));
            B2 = L1/theta1*sin(theta1)+L0;
            C1 = L1/theta1*sin(theta1);
            C2 = L1/theta1*cos(theta1)+L0/theta1;
            Jv = [-B1*cos(delta)/theta1 + C1*cos(delta) -B1*sin(delta); ...
                  -B1*sin(delta)/theta1 + C1*sin(delta) B1*cos(delta); ...
                  -B2/theta1+C2 0];
        end
    end
end
end