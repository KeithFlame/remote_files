function [t,y]=odeCosserat_inverse(y0,v,SegIdx,MP,fe,le,step)
%-----Integral of IVP using difference equation-----
% for BottumUPCosserat_TwoSeg_inv script, inverse differential equations.
% y0- initial (tip) condition
% v- rod elongation strain
% SegIdx- the segment index (1 or 2) you are integrating
% MP- Mechanical properties
% fe, le- external distributed loads
% step- integrating step size
%----------info-----------%
% ver i1.0
% by Yuyang Chen
% date 20200604
%-------------------------------------------------------------------------%
DoF=length(y0);
if(SegIdx == 1)
    Q=MP.Q1;
    N=round(MP.L1/step)+1;
    t=linspace(floor(MP.L1*1e3),0,N-1)';
    t=t*1e-3;
    t=t+MP.ell;
    t=[MP.L1+MP.ell;t];
    Kb = 4*MP.Kb1+4*MP.Kb2;
    mod_SegIdx=1;
elseif(SegIdx == 2)
    Q=MP.Q2;
    N=round(MP.L2/step)+1;
    t=linspace(floor(MP.L2*1e3),0,N)';
    t=t*1e-3;
    t=t+MP.L1+MP.Lr+MP.ell;
    Kb = 4*MP.Kb2;
    mod_SegIdx=0;
elseif(SegIdx == 0)%for base stem
    Q=MP.Q1;
    N=floor(MP.ell/step)+2;
    t=linspace(floor(MP.ell/1e-3),0,N-1)';
    t=t*1e-3;
    t=[MP.ell;t];
    gamma=4.2;
    Kb = 4*MP.Kb1+4*MP.Kb2 + (gamma-1)*8*MP.Kb2;
    mod_SegIdx=1;
end

y=zeros(N,DoF);
y(1,:)=y0';
for i=1:N-1
    p=y(i,1:3)';R=[y(i,4:6);y(i,7:9);y(i,10:12)]';n=y(i,13:15)';m=y(i,16:18)';q=y(i,19:end)';
    u=inv(Kb)*R'*m;u(3)=0;
    
    %constant-curvature-based evolution
    theta=(t(i)-t(i+1))*norm(u);delta=-atan2(u(2),u(1))+pi/2;
    if(theta~=0)
        p_dot=R*( (t(i)-t(i+1))/theta*[cos(delta)*(1-cos(theta)) sin(delta)*(cos(theta)-1) -sin(theta)]' );
        R_dot=[cos(delta)^2*cos(theta)+sin(delta)^2 -sin(delta)*cos(delta)*(cos(theta)-1) cos(delta)*sin(theta);...
               sin(delta)*cos(delta)*(1-cos(theta)) cos(delta)^2+cos(theta)*sin(delta)^2 -sin(delta)*sin(theta);...
              -cos(delta)*sin(theta) sin(delta)*sin(theta) cos(theta)];
    else
        p_dot=R*[0 0 -(t(i)-t(i+1))]';
        R_dot=eye(3);
    end
    p=p+(p_dot);
    R=R*R_dot';

    n_dot=fe*(t(i)-t(i+1));
    m_dot=-S(p_dot)*n + le*(t(i)-t(i+1)) +...
    2*R*(( cross(S(u)*MP.r11,MP.Ke1*v(:,1))+cross(MP.r11,S(u)*MP.Ke1*v(:,1)) + ...
           cross(S(u)*MP.r12,MP.Ke1*v(:,2))+cross(MP.r12,S(u)*MP.Ke1*v(:,2)) )*mod_SegIdx + ...
         ( cross(S(u)*MP.r21,MP.Ke2*v(:,3))+cross(MP.r21,S(u)*MP.Ke2*v(:,3)) + ...
           cross(S(u)*MP.r22,MP.Ke2*v(:,4))+cross(MP.r22,S(u)*MP.Ke2*v(:,4)) ))*(t(i)-t(i+1));
    n=n+n_dot;
    m=m+m_dot;
    
    q_dot=-Q'*u*(t(i)-t(i+1));
    q=q+q_dot;
    y(i+1,:)=[p;R(:,1);R(:,2);R(:,3);n;m;q]';
end

end