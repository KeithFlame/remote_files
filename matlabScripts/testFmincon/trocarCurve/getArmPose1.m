function T0=getArmPose1(l,theta,delta,T)
    if(l<1e-9||theta<1e-6)
        [T0,~]=plotSeg(0,0,0,T);
    else
        [T0,~]=plotSeg(l,theta,delta,T);
    end
end