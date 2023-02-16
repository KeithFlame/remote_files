function u = Psi2Curvature_keith(psi,SL)
% this is afunction to get curvature of a 2-seg continuum manipulator form
% a known configuration variable and several structural parameters.
%
% Author: Keith W.
% Ver. 1.0
% Date: 15.02.2022

    uc=zeros(6,1);
    l1 = psi(2);
    zeta = SL(8);
    if(SL(1)<l1)
        L1=SL(1)*1e-3;
    else
        L1= l1*1e-3;
    end
    L2=SL(3)*1e-3;
    
    if size(psi,2)==4
        temPsi=[0 0 psi];
    else
        temPsi=psi;
    end
    if(l1<=SL(1))
        theta1=temPsi(3);
        theta2=temPsi(5);
        delta1=temPsi(4);
        delta2=temPsi(6);
    else
        theta1=L1/(zeta*(l1*1e-3-L1)+L1)*temPsi(3);
        theta2=temPsi(5);
        delta1=temPsi(4);
        delta2=temPsi(6);
    end 
    uc(1)=theta1/L1*cos(pi/2+delta1);
    uc(2)=theta1/L1*sin(pi/2+delta1);
    uc(4)=theta2/L2*cos(pi/2+delta2);
    uc(5)=theta2/L2*sin(pi/2+delta2);
    u=uc;

end

