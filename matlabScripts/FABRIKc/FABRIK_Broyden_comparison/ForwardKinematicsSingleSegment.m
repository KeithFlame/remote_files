function T = ForwardKinematicsSingleSegment(theta, delta,L)

    k=theta/L;
    
    if theta==0
        T=[1 0 0 0
            0 1 0 0
            0 0 1 L1
            0 0 0 1];
    else
        cosTHETA1=cos(theta);sinTHETA1=sin(theta);cosDELTA1=cos(delta);sinDELTA1=sin(delta);
        T=[(cosDELTA1)^2*(cosTHETA1-1)+1 sinDELTA1*cosDELTA1*(cosTHETA1-1) cosDELTA1*sinTHETA1 cosDELTA1*(1-cosTHETA1)/k
            sinDELTA1*cosDELTA1*(cosTHETA1-1) (cosDELTA1)^2*(1-cosTHETA1)+cosTHETA1 sinDELTA1*sinTHETA1 sinDELTA1*(1-cosTHETA1)/k
            -cosDELTA1*sinTHETA1 -sinDELTA1*sinTHETA1 cosTHETA1 sinTHETA1/k
            0 0 0 1];
    end
end