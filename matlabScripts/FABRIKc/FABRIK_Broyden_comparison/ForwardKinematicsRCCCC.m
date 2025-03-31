function T = ForwardKinematicsRCCCC(psi,SL)
    phi = psi(1);
    theta1 = psi(2); delta1 = psi(3);
    theta2 = psi(4); delta2 = psi(5);
    theta3 = psi(6); delta3 = psi(7);
    theta4 = psi(8); delta4 = psi(9);
    T0 = [cos(phi) -sin(phi) 0 0;sin(phi) cos(phi) 0 0; 0 0 1 0; 0 0 0 1];
    T1 = ForwardKinematicsSingleSegment(theta1, delta1,SL(1));
    T2 = ForwardKinematicsSingleSegment(theta2, delta2,SL(2));
    T3 = ForwardKinematicsSingleSegment(theta3, delta3,SL(3));
    T4 = ForwardKinematicsSingleSegment(theta4, delta4,SL(4));
    T = T0*T1*T2*T3*T4;
end