function [t,y]=odeCosserat_single(y0,v,MP,fe,le,step,Len)
%-----Integral of IVP using difference equation-------
% for BottumUPCosserat_TwoSeg script, forward differential equations.
% y0- initial condition
% v- rod elongation strain
% SegIdx- the segment index (1 or 2) you are integrating
% MP- Mechanical properties
% fe, le- external distributed loads
% step- integrating step size
%----------info-----------%
% ver f1.0
% by Yuyang Chen
% date 20201229
%-------------------------------------------------------------------------%
DoF=length(y0);
    Q=MP.Q1;
    N=round((Len(2)-Len(1))/step)+1;
    t=linspace(Len(1),Len(1)+(N-2)*1e-3,N-1)';
    t(end+1) = Len(2);
    Kb1=MP.Kb1;
    Ke1=MP.Ke1;

y=zeros(N,DoF);
y(1,:)=y0';
for i=1:N-1
    step=t(i+1)-t(i);
    p=y(i,1:3)';R=[y(i,4:6);y(i,7:9);y(i,10:12)]';n=y(i,13:15)';m=y(i,16:18)';q=y(i,19:end)';
    u=inv(4*Kb1)*R'*m;u(3)=0;
    
    %constant-curvature-based evolution
%     theta=step*norm(u);delta=-atan2(u(2),u(1))+pi/2;
%     if(theta~=0)
%         p_dot=R*( step/theta*[cos(delta)*(1-cos(theta)) sin(delta)*(cos(theta)-1) sin(theta)]' );
%         R_dot=[cos(delta)^2*cos(theta)+sin(delta)^2 -sin(delta)*cos(delta)*(cos(theta)-1) cos(delta)*sin(theta);...
%                sin(delta)*cos(delta)*(1-cos(theta)) cos(delta)^2+cos(theta)*sin(delta)^2 -sin(delta)*sin(theta);...
%               -cos(delta)*sin(theta) sin(delta)*sin(theta) cos(theta)];
%     else
%         p_dot=R*[0 0 step]';
%         R_dot=eye(3);
%     end
    p_dot = R*[0 0 1]'*step;
    R_dot = Expm(u*step);
    
    p=p+p_dot;
    R=R*R_dot;

    n_dot=-fe*step;
    m_dot=-S(p_dot)*n - le*step -...
    2*R*( cross(S(u)*MP.r11,Ke1*v(:,1))+cross(MP.r11,S(u)*Ke1*v(:,1)) + ...
          cross(S(u)*MP.r12,Ke1*v(:,2))+cross(MP.r12,S(u)*Ke1*v(:,2)) )*step;
    n=n+n_dot;
    m=m+m_dot;
    
    q_dot=Q'*u*step;
    q=q+q_dot;
    y(i+1,:)=[p;R(:,1);R(:,2);R(:,3);n;m;q]';
end

end