function [t,y,U,ee]=odeCosserat_keith(y0,v,SegIdx,ksi,MBP)
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
Kb1=MBP.Kb1;
Ke1=MBP.Ke1;
Kb2=MBP.Kb2;
Ke2=MBP.Ke2;
m_ro=0;
Gm=9.8;
Ke=eye(3);
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
    m_ro=ksi(5);
    step=l_do/(N-1);
end
    
y=y0';
% R=[y(1,4:6);y(1,7:9);y(1,10:12)]';m=y(1,16:18)';
es0=0;eb=0;es=0;
Kb_=inv(Kb);
Ke_=inv(Ke);
r11 = MBP.r11;
r12 = MBP.r12;
r21 = MBP.r21;
r22 = MBP.r22;
for i=1:N-1
    p=y(1:3)';R=[y(4:6);y(7:9);y(10:12)]';n=y(13:15)';m=y(16:18)';q=y(19:end)';
    v_do = 0;
    %constant-curvature-based evolution
    if(SegIdx<3)
        u=Kb_*R'*m;u(3)=0;
        theta=step*norm(u);delta=-atan2(u(2),u(1))+pi/2;
        [p_dot,R_dot] = calcRdot(theta,delta,R,step);
        p=p+p_dot;
        R=R*R_dot;    
        n_dot=0;
        
        m_dot=-cross(p_dot,n) -...
        2*R*( ( ...
               cross(cross(u,r11),Ke1*v(:,1))+cross(r11,cross(u,Ke1*v(:,1))) + ...
               cross(cross(u,r12),Ke1*v(:,2))+cross(r12,cross(u,Ke1*v(:,2))) )*mod_SegIdx + ...
             ( cross(cross(u,r21),Ke2*v(:,3))+cross(r21,cross(u,Ke2*v(:,3))) + ...
               cross(cross(u,r22),Ke2*v(:,4))+cross(r22,cross(u,Ke2*v(:,4))) ))*step;
        n=n+n_dot;
        m=m+m_dot;
        
        q_dot=Q'*u*step;
        q=q+q_dot;
    else
        u=Kb_*R'*m;
        vs3 = Ke_*R'*n;
        v_do = vs3(3)*step;
        theta=step*norm(u(1:2));delta=-atan2(u(2),u(1))+pi/2;phi = step * u(3);
        [p_dot,R_dot] = calcRdot(theta,delta,R,step+v_do);
        R_phi=[cos(phi) -sin(phi) 0; sin(phi) cos(phi) 0; 0 0 1];
        R_dot = R_dot*R_phi;
        p=p+p_dot;
        R=R*R_dot;
    
        n_dot=-step*m_ro*[0 0 -1]'*Gm;
        m_dot =-cross(p_dot,n);
        n=n+n_dot;
        m=m+m_dot;
        
        
        q=q+v_do;
        vs=[0 0 vs3(3)]';
        es0 = es0+step*(vs-[0;0;0])'*Ke*(vs-[0;0;0]);
    end
    eb = eb+u'*Kb*u*(step+v_do);
    
    y=[p;R(:,1);R(:,2);R(:,3);n;m;q]';
    U=u';
end
    l_1 = MBP.Lstem+MBP.L1;
    l_2 = l_1+MBP.Lr+MBP.L2;
    if(SegIdx == 1)
        es =l_1*(v(:,1)-[0;0;0])'*Ke1*(v(:,1)-[0;0;0])*2+...
            l_1*(v(:,2)-[0;0;0])'*Ke1*(v(:,2)-[0;0;0])*2;
    elseif(SegIdx == 2)
        es =l_2*(v(:,3)-[0;0;0])'*Ke2*(v(:,3)-[0;0;0])*8+...
            l_2*(v(:,4)-[0;0;0])'*Ke2*(v(:,4)-[0;0;0])*8;
    elseif(SegIdx == 3)
        es = es0;
    else
        es = 0;
    end
% U(end,:)=Kb\R'*m;
ee=[eb es]';
[n,~]=size(y);
try
    assert(n==1);
catch
    warning('Problem using function.  Assigning a value of 0.');
end    
end


function [p_dot,R_dot] = calcRdot(theta,delta,R,step)
    if(abs(theta)>0)
        costheta=cos(theta);sintheta=sin(theta);cosdelta=cos(delta);sindelta=sin(delta); % cache
        p_dot=R*( step/theta*[cosdelta*(1-costheta) sindelta*(costheta-1) sintheta]' );
        R_dot=[cosdelta^2*costheta+sindelta^2 -sindelta*cosdelta*(costheta-1) cosdelta*sintheta;...
           sindelta*cosdelta*(1-costheta) cosdelta^2+costheta*sindelta^2 -sindelta*sintheta;...
          -cosdelta*sintheta sindelta*sintheta costheta];
    else
        R_dot = eye(3);
        p_dot=R*[0 0 step]';
    end
end
