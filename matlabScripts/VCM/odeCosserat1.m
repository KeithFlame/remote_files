function [t,y,U]=odeCosserat1(y0,v,SegIdx,MP,fe,le,step)
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
% date 20200524
%-------------------------------------------------------------------------%
DoF=length(y0);
Kb1=MP.Kb1;
Ke1=MP.Ke1;
Kb2=MP.Kb2;
Ke2=MP.Ke2;
if(SegIdx == 1)
    Q=MP.Q1;
    N=round(MP.L1/step)+1;
    t=linspace(0,MP.L1,N)';
    mod_SegIdx=1;
    Kb = 4*Kb1+4*Kb2;
    step=MP.L1/(N-1);
elseif(SegIdx == 2)
    Q=MP.Q2;
    N=round(MP.L2/step)+1;
    t=linspace(MP.L1+MP.Lr,MP.L1+MP.Lr+MP.L2,N)';
    mod_SegIdx=0;
    Kb = 4*Kb2;
    step=MP.L2/(N-1);
elseif(SegIdx == 0)
    Q=MP.Q1;
    N=round(MP.ell/step)+1;
    t=linspace(0,MP.ell,N)';
    mod_SegIdx=1;
%     gamma=4.2;
    gamma=1;
    Kb = 4*Kb1+4*Kb2 + (gamma-1)*8*Kb2;
    step=MP.ell/(N-1);
end
    
y=zeros(N,DoF);U=zeros(N,3);
y(1,:)=y0';R=[y0(4:6) y0(7:9) y0(10:12)];m=y0(16:18);
for i=1:N-1
    p=y(i,1:3)';R=[y(i,4:6);y(i,7:9);y(i,10:12)]';n=y(i,13:15)';m=y(i,16:18)';q=y(i,19:end)';
%     u=(4*Kb2+mod(SegIdx,2)*4*Kb1)\R'*m;u(3)=0; % torsion free !!!
    u=Kb\R'*m;u(3)=0;
    
    %constant-curvature-based evolution
    theta=step*norm(u);delta=-atan2(u(2),u(1))+pi/2;
    costheta=cos(theta);sintheta=sin(theta);cosdelta=cos(delta);sindelta=sin(delta); % cache
    if(theta~=0)
        p_dot=R*( step/theta*[cosdelta*(1-costheta) sindelta*(costheta-1) sintheta]' );
        R_dot=[cosdelta^2*costheta+sindelta^2 -sindelta*cosdelta*(costheta-1) cosdelta*sintheta;...
               sindelta*cosdelta*(1-costheta) cosdelta^2+costheta*sindelta^2 -sindelta*sintheta;...
              -cosdelta*sintheta sindelta*sintheta costheta];
    else
        p_dot=R*[0 0 step]';
        R_dot=eye(3);
    end
    p=p+p_dot;
    R=R*R_dot;

    n_dot=-fe*step;
    m_dot=-S(p_dot)*n - le*step -...
    2*R*(( cross(S(u)*MP.r11,Ke1*v(:,1))+cross(MP.r11,S(u)*Ke1*v(:,1)) + ...
           cross(S(u)*MP.r12,Ke1*v(:,2))+cross(MP.r12,S(u)*Ke1*v(:,2)) )*mod_SegIdx + ...
         ( cross(S(u)*MP.r21,Ke2*v(:,3))+cross(MP.r21,S(u)*Ke2*v(:,3)) + ...
           cross(S(u)*MP.r22,Ke2*v(:,4))+cross(MP.r22,S(u)*Ke2*v(:,4)) ))*step;
    n=n+n_dot;
    m=m+m_dot;
    
    q_dot=Q'*u*step;
    q=q+q_dot;
    y(i+1,:)=[p;R(:,1);R(:,2);R(:,3);n;m;q]';
    U(i,:)=u';
end
U(end,:)=Kb\R'*m;
end