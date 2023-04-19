function [t,y,U]=odeCosserat_keith(y0,v,SegIdx,ksi)
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
% by Yuyang Chen
% date 20200524
%-------------------------------------------------------------------------%
DoF=length(y0);
Kb1=[    0.0016         0         0
         0    0.0016         0
         0         0    0.0012];
Ke1=1e4*[    1.0659         0         0
         0    1.0659         0
         0         0    2.8353];
Kb2=1e-4*[    0.5027         0         0
         0    0.5027         0
         0         0    0.3779];
Ke2=1e4*[    0.7559         0         0
         0    0.7559         0
         0         0    2.0106];
L1o = double(60e-3);
Lr = double(10e-3);
L2 = double(20e-3);
K1 = double(5);
K2 = double(0.6);
Ls = double(0);
zeta = double(0.2);
Ldo = 20e-3;
r12=[0.0000    0.0025         0]';
r11=[ 0.0025 0 0]';
r22=[0.0019    0.0019         0]';
r21=[0.0019   -0.0019         0]';
fe = 0;
le = 0;
step = double(1e-3);
N=int32(0);
mod_SegIdx = int8(0);
Kb = eye(3);
Ke = eye(3);
Q = [];
t=[];
if(SegIdx == 1)
    Q=[0    0.0025   -0.0019    0.0019
   -0.0025   -0.0000   -0.0019   -0.0019
         0         0         0         0];
    N=int32(ceil(L1o/step));
    t=linspace(0,L1o,N)';
    mod_SegIdx=int8(1);
    Kb = 4*Kb1+16*Kb2+ K1 * Kb1;
    step=L1o/double(N-1);
elseif(SegIdx == 2)
    Q=[   -0.0019    0.0019
   -0.0019   -0.0019
         0         0];
    N=int32(ceil(L2/step));
    t=linspace(L1o+Lr,L1o+Lr+L2,N)';
    mod_SegIdx=int8(0);
    Kb = 16*Kb2+K2 * Kb1;
    step=L2/double(N-1);
elseif(SegIdx == 0)
    Q=[0    0.0025   -0.0019    0.0019
   -0.0025   -0.0000   -0.0019   -0.0019
         0         0         0         0];
    N=int32(ceil(Ls/step));
    t=linspace(0,Ls,N)';
    mod_SegIdx=int8(1);
    Kb = (4*Kb1+16*Kb2+K1 * Kb1)/zeta;
    step=Ls/double(N-1);
elseif(SegIdx == 3)
    l_do =Ldo;
    N=int32(ceil(l_do/step));
    t=linspace(0,l_do,N)';
    mod_SegIdx = int8(3);
    Kb = diag(ksi(2:4));
    Ke = diag([10000,10000,1]*ksi(1));
    step=l_do/double(N-1);
end
    
y=zeros(N,DoF);U=zeros(N,3);
y(1,:)=y0';R=[y0(4:6) y0(7:9) y0(10:12)];m=y0(16:18);
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
    
        n_dot=-fe*step;
        
        m_dot=-skewMatrix_keith(p_dot)*n - le*step -...
        2*R*((cross(skewMatrix_keith(u)*r11,Ke1*v(:,1))+cross(r11,skewMatrix_keith(u)*Ke1*v(:,1)) + ...
               cross(skewMatrix_keith(u)*r12,Ke1*v(:,2))+cross(r12,skewMatrix_keith(u)*Ke1*v(:,2)) )*double(mod_SegIdx) + ...
             ( cross(skewMatrix_keith(u)*r21,Ke2*v(:,3))+cross(r21,skewMatrix_keith(u)*Ke2*v(:,3)) + ...
               cross(skewMatrix_keith(u)*r22,Ke2*v(:,4))+cross(r22,skewMatrix_keith(u)*Ke2*v(:,4)) ))*step;
        n=n+n_dot;
        m=m+m_dot;
        
        q_dot=Q'*u*step;
        q=q+q_dot;
    else
        u=Kb\R'*m;
        v_ = Ke\R'*n;
        v_do = v_(3)*step;
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
    
        n_dot=-fe*step;
        m_dot =-skewMatrix_keith(p_dot)*n - le*step;
        n=n+n_dot;
        m=m+m_dot;
        
        
        q=q+v_do;
    end

    y(i+1,:)=[p;R(:,1);R(:,2);R(:,3);n;m;q]';
    U(i,:)=u';
end
U(end,:)=Kb\R'*m;
end
