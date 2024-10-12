function u = Psi2Curvature_keith(psi,SL)
% this is afunction to get curvature of a 2-seg continuum manipulator form
% a known configuration variable and several structural parameters.
%
% input1: psi (6 X 1 vector, phi L theta1 delta1 theta2 delta2) (mm, rad)
% input2: structural length SL (4 X 1 vector, L1 Lr L2 Lg) (mm)
%                           or (10 X 1 vector additional zeta K1 K2 gamma1
%                           Lstem gamma3) (mm, rad, or dimensionless)
%
% output u curvature of the 2-seg continuum manipulator (6 X 1 vector) (1/m)
% Author: Keith W.
% Ver. 1.0
% Date: 15.02.2022

    if(length(SL)<5)
        zeta = 0.2;
    else
        zeta = SL(5);
    end
    uc=zeros(6,1);
    l1 = psi(2);
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

