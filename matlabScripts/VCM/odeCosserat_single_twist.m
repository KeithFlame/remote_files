function [t,y]=odeCosserat_single_twist(y0,v,MP,fe,le,step,Len)
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
    %Q=MP.Q1;
    N=round((Len(2)-Len(1))/step)+1;
    t=linspace(Len(1),Len(1)+(N-2)*1e-3,N-1)';
    t(end+1) = Len(2);
    Kb1=MP.Kb1;
    Ke1=MP.Ke1;
    Ke0=MP.Ke0;
    Kb0=MP.Kb0;

y=zeros(N,DoF);
y(1,:)=y0';
for i=1:N-1
    step=t(i+1)-t(i);
    p=y(i,1:3)';R=[y(i,4:6);y(i,7:9);y(i,10:12)]';n=y(i,13:15)';m=y(i,16:18)';q_diff=y(i,19:20)';q_comm=y(i,21);
    v_tilde=inv(4*Ke1+Ke0)*R'*n;
    u=inv(4*Kb1+Kb0)*R'*m;
      %   -diag([0 0 2])*S(MP.r11)*Ke1*S(MP.r11) ...
      %   -diag([0 0 2])*S(MP.r12)*Ke1*S(MP.r12) ...

    p_dot = R*(v_tilde+[0 0 1]')*step;
    R_dot = Expm(u*step);
    
    p=p+p_dot;
    R=R*R_dot;

    n_dot=-fe*step;
    m_dot=-S(p_dot)*n - le*step -...
    2*R*( cross(S(u)*MP.r11,Ke1*v(:,1))+cross(MP.r11,S(u)*Ke1*v(:,1)) + ...
          cross(S(u)*MP.r12,Ke1*v(:,2))+cross(MP.r12,S(u)*Ke1*v(:,2)) )*step;
    n=n+n_dot;
    m=m+m_dot;
    
    q_dot_comm=0.5*norm([0 0 1]'+v_tilde+cross(u,MP.r11))+0.5*norm([0 0 1]'+v_tilde-cross(u,MP.r11))-1;
    %q_dot_comm2=0.5*norm([0 0 1]'+v_tilde+cross(u,MP.r12))+0.5*norm([0 0 1]'+v_tilde-cross(u,MP.r12))-1;
    q_dot=[norm([0 0 1]'+cross(u,MP.r11))-1;
           norm([0 0 1]'+cross(u,MP.r12))-1];
    q_dot_diff=[q_dot(1)-q_dot_comm;q_dot(2)-q_dot_comm];
    q_diff=q_diff+q_dot_diff*step;
    q_comm=q_comm+q_dot_comm*step;
    
    y(i+1,:)=[p;R(:,1);R(:,2);R(:,3);n;m;q_diff;q_comm]';
end

end