function [t,y]=odeCosserat1_twist(y0,v,SegIdx,MP,fe,le,step)
%-----Integral of IVP using difference equation-------
% for BottumUPCosserat_TwoSeg script, forward differential equations.
% y0- initial condition
% v- rod elongation strain 3x4?
% SegIdx- the segment index (1 or 2) you are integrating
% MP- Mechanical properties
% fe, le- external distributed loads
% step- integrating step size
%----------info-----------%
% ver f1.0
% by Yuyang Chen
% date 20200524
%-------------------------------------------------------------------------%
DoF=length(y0);
if(SegIdx == 1)
    Q=MP.Q1; % coupling matrix?
    N=round(MP.L1/step)+1; % number of points
    t=linspace(0,floor(MP.L1*1e3),N-1)';
    t=t*1e-3; % meter to milimeter
    t=t+MP.ell; % ?
    t=[t;MP.L1+MP.ell]; % length of the points s ?
    Kb = 4*MP.Kb1+4*MP.Kb2+MP.Kb0;
    Ke = 4*MP.Ke1+4*MP.Ke2+MP.Ke0;
    mod_SegIdx=1;
elseif(SegIdx == 2)
    Q=MP.Q2;
    N=round(MP.L2/step)+1;
    t=linspace(0,floor(MP.L2*1e3),N)';
    t=t*1e-3;
    t=t+MP.L1+MP.Lr+MP.ell;
    Kb = 4*MP.Kb2+MP.Kb0;
    Ke = 4*MP.Ke2+MP.Ke0;
    mod_SegIdx=0;
elseif(SegIdx == 0)%for base stem
    Q=MP.Q1;
    N=floor(MP.ell/step)+2;
    t=linspace(0,floor(MP.ell/1e-3),N-1)';
    t=t*1e-3;
    t=[t;MP.ell];
    Kb = 4*MP.Kb1+4*MP.Kb2+MP.Kbs;
    Ke = 4*MP.Ke1+4*MP.Ke2+MP.Kes;
    mod_SegIdx=1;
end


y=zeros(N,DoF); % every row is a point
y(1,:)=y0'; % initial values
for i=1:N-1 % point i is known, calculate point i+1
    p=y(i,1:3)';R=[y(i,4:6);y(i,7:9);y(i,10:12)]';n=y(i,13:15)';m=y(i,16:18)';q_comm=y(i,19);q_diff=y(i,20:end)'; % state variables
%     v_tilde=inv(Ke)*R'*n; % internal force
%     u=inv(Kb)*R'*m; % internal moment
    v_tilde=Ke\R'*n; % constitutive law
    u=Kb\R'*m;
    
    %constant-curvature-based evolution !!!
    p_dot = R*(v_tilde+[0 0 1]')*(t(i+1)-t(i)); % t(i+1)-t(i)
    R_dot = Expm(u*(t(i+1)-t(i)));
    % these are actually delta_p and delta_R
    
    p=p+p_dot; % pose at t(i+1)
    R=R*R_dot;

    n_dot=-fe*(t(i+1)-t(i));
    m_dot=-S(p_dot)*n - le*(t(i+1)-t(i)) -...
    2*R*(( cross(S(u)*MP.r11,MP.Ke1*v(:,1))+cross(MP.r11,S(u)*MP.Ke1*v(:,1)) + ...
           cross(S(u)*MP.r12,MP.Ke1*v(:,2))+cross(MP.r12,S(u)*MP.Ke1*v(:,2)) )*mod_SegIdx + ...
         ( cross(S(u)*MP.r21,MP.Ke2*v(:,3))+cross(MP.r21,S(u)*MP.Ke2*v(:,3)) + ...
           cross(S(u)*MP.r22,MP.Ke2*v(:,4))+cross(MP.r22,S(u)*MP.Ke2*v(:,4)) ))*(t(i+1)-t(i));
    n=n+n_dot;
    m=m+m_dot;
    
    q_dot_comm=0.5*norm([0 0 1]'+v_tilde+cross(u,MP.r11))+0.5*norm([0 0 1]'+v_tilde-cross(u,MP.r11))-1; %r12
    %q_dot_comm2=0.5*norm([0 0 1]'+v_tilde+cross(u,MP.r12))+0.5*norm([0 0 1]'+v_tilde-cross(u,MP.r12))-1;
    if(SegIdx == 2)
        q_dot=[norm([0 0 1]'+cross(u,MP.r21))-1;
               norm([0 0 1]'+cross(u,MP.r22))-1];
    else
        q_dot=[norm([0 0 1]'+cross(u,MP.r11))-1;
               norm([0 0 1]'+cross(u,MP.r12))-1;
               norm([0 0 1]'+cross(u,MP.r21))-1;
               norm([0 0 1]'+cross(u,MP.r22))-1];
    end
    q_dot_diff=q_dot-q_dot_comm;
    q_diff=q_diff+q_dot_diff*(t(i+1)-t(i));
    q_comm=q_comm+q_dot_comm*(t(i+1)-t(i));
    
    y(i+1,:)=[p;R(:,1);R(:,2);R(:,3);n;m;q_comm;q_diff]';
end

end