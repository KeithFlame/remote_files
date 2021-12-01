function T=getTrocarPose1(d)

    % set trocar para 
    SP=getSP;
    arc_radius=SP.trocar.arc_radius;
    theta=d/arc_radius;
    delta=0;
    [T,~]=plotSeg(-d,theta,delta,eye(4));
end