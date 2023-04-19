function [t,y,U]=odeCosserat_keith(y0,v,SegIdx,ksi,MBP)
%-----Integral of IVP using difference equation-------
% for a coninuum robot kinematics script, forward differential equations.
% 
% input1: y0 initial condition
% input2: v rod elongation strain
% SegIdx- the segment index (1 or 2) you are integrating
% MBP- Mechanical properties
% fe, le- external distributed loads
% step- integrating step size
%----------info-----------%
% ver f1.0
% by Keith W.
% date 04.14.2023
%-------------------------------------------------------------------------%
DoF=length(y0);
Kb1=MBP.Kb1;
Ke1=MBP.Ke1;
Kb2=MBP.Kb2;
Ke2=MBP.Ke2;
if(SegIdx == 1)
    Q=MBP.Q1;
    N=ceil(MBP.L1o/MBP.discrete_element);
    t=linspace(0,MBP.L1o,N)';
    mod_SegIdx=1;
    Kb = 4*Kb1+16*Kb2+ MBP.K1 * Kb1;
    step=MBP.L1o/(N-1);
elseif(SegIdx == 2)
    Q=MBP.Q2;
    N=ceil(MBP.L2/MBP.discrete_element);
    t=linspace(MBP.L1o+MBP.Lr,MBP.L1o+MBP.Lr+MBP.L2,N)';
    mod_SegIdx=0;
    Kb = 16*Kb2+MBP.K2 * Kb1;
    step=MBP.L2/(N-1);
elseif(SegIdx == 0)
    Q=MBP.Q1;
    N=ceil(MBP.Ls/MBP.discrete_element);
    t=linspace(0,MBP.Ls,N)';
    mod_SegIdx=1;
    zeta=MBP.zeta;
    Kb = (4*Kb1+16*Kb2+MBP.K1 * Kb1)/zeta;
    step=MBP.Ls/(N-1);
elseif(SegIdx == 3)
    l_do = MBP.Ldo;
    N=ceil(l_do/MBP.discrete_element);
    t=linspace(0,l_do,N)';
    mod_SegIdx = 3;
    Kb = diag(ksi(2:4));
    Ke = diag([10000,10000,1]*ksi(1));
    step=l_do/(N-1);
end
    
y=zeros(N,DoF);U=zeros(N,3);
y(1,:)=y0';
for i=1:N-1
    p=y(i,1:3)';R=[y(i,4:6);y(i,7:9);y(i,10:12)]';n=y(i,13:15)';m=y(i,16:18)';q=y(i,19:end)';
    
    %constant-curvature-based evolution
    if(SegIdx<3)
        u=Kb\R'*m;u(3)=0;
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
    
        n_dot=0;
        
        m_dot=-skewMatrix_keith(p_dot)*n -...
        2*R*( ...
        ( ...
        cross( ...
            skewMatrix_keith(u)*MBP.r11,Ke1*v(:,1) ...
            )+cross( ...
                MBP.r11,skewMatrix_keith(u)*Ke1*v(:,1) ...
                    ) + ...
               cross(skewMatrix_keith(u)*MBP.r12,Ke1*v(:,2))+cross(MBP.r12,skewMatrix_keith(u)*Ke1*v(:,2)) )*mod_SegIdx + ...
             ( cross(skewMatrix_keith(u)*MBP.r21,Ke2*v(:,3))+cross(MBP.r21,skewMatrix_keith(u)*Ke2*v(:,3)) + ...
               cross(skewMatrix_keith(u)*MBP.r22,Ke2*v(:,4))+cross(MBP.r22,skewMatrix_keith(u)*Ke2*v(:,4)) ))*step;
        n=n+n_dot;
        m=m+m_dot;
        
        q_dot=Q'*u*step;
        q=q+q_dot;
    else
        u=Kb\R'*m;
        v = Ke\R'*n;
        v_do = v(3)*step;
        theta=step*norm(u(1:2));delta=-atan2(u(2),u(1))+pi/2;phi = step * u(3);
        costheta=cos(theta);sintheta=sin(theta);cosdelta=cos(delta);sindelta=sin(delta); % cache
        if(theta~=0)
            p_dot=R*( (step+v_do)/theta*[cosdelta*(1-costheta) sindelta*(costheta-1) sintheta]' );
            R_dot=[cosdelta^2*costheta+sindelta^2 -sindelta*cosdelta*(costheta-1) cosdelta*sintheta;...
                   sindelta*cosdelta*(1-costheta) cosdelta^2+costheta*sindelta^2 -sindelta*sintheta;...
                  -cosdelta*sintheta sindelta*sintheta costheta]*eul2rotm([phi 0 0]);
        else
            p_dot=R*[0 0 (step+v_do)]';
            R_dot=eye(3)*eul2rotm([phi 0 0]);
        end
        p=p+p_dot;
        R=R*R_dot;
    
        n_dot=0;
        m_dot =-skewMatrix_keith(p_dot)*n;
        n=n+n_dot;
        m=m+m_dot;
        
        
        q=q+v_do;
    end

    y(i+1,:)=[p;R(:,1);R(:,2);R(:,3);n;m;q]';
    U(i,:)=u';
end
U(end,:)=Kb\R'*m;
end
