function T = ForwardKinematicsRUUUU(psi,SL)
    phi = psi(1);
    theta1 = psi(2); delta1 = psi(3);
    theta2 = psi(4); delta2 = psi(5);
    theta3 = psi(6); delta3 = psi(7);
    theta4 = psi(8); delta4 = psi(9);
    T0 = [cos(phi) -sin(phi) 0 0;sin(phi) cos(phi) 0 0; 0 0 1 0; 0 0 0 1];
    T1 = ForwardKinematicsUniversal(theta1, delta1,SL(1));
    T2 = ForwardKinematicsUniversal(theta2, delta2,SL(2));
    T3 = ForwardKinematicsUniversal(theta3, delta3,SL(3));
    T4 = ForwardKinematicsUniversal(theta4, delta4,SL(4));
    T = T0*T1*T2*T3*T4;
end

function T = ForwardKinematicsUniversal(theta, delta,L)

    k=theta/L;
    
    if theta==0
        T=[1 0 0 0
            0 1 0 0
            0 0 1 L
            0 0 0 1];
    else
        cosTHETA1=cos(theta);sinTHETA1=sin(theta);cosDELTA1=cos(delta);sinDELTA1=sin(delta);
        T=[(cosDELTA1)^2*(cosTHETA1-1)+1 sinDELTA1*cosDELTA1*(cosTHETA1-1) cosDELTA1*sinTHETA1 0
            sinDELTA1*cosDELTA1*(cosTHETA1-1) (cosDELTA1)^2*(1-cosTHETA1)+cosTHETA1 sinDELTA1*sinTHETA1 0
            -cosDELTA1*sinTHETA1 -sinDELTA1*sinTHETA1 cosTHETA1 0
            0 0 0 1];
        T(1:3,4)=T(1:3,1:3)*[0 0 L]';
    end
end