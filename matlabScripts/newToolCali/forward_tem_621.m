function [kmp]=forward_tem_621(psi,seg,zeta,d,ga)
L1x=0;
%=====psi = {l, phi, theta1, delta1, theta2, delta2}
l=psi(1)+d+L1x;
phi=psi(2);
theta1=psi(3);
delta1=psi(4);
theta2=psi(5);
delta2=psi(6);
%====mechanics based coupling effects:parameters
length = [max(0,l-seg(1)-seg(2)-seg(3)) ...
          min(max(0,l-seg(2)-seg(3)),seg(1)) ...
          min(max(0,l-seg(3)),seg(2)) ...
          min(l,seg(3))]';
L0=length(1);
L1=length(2);
Lr=length(3);
L2=length(4);
Lg=seg(4);
if(L1<1e-3)
    Theta0 = 0;
    Theta1 = 0;
else
    Theta0 = theta1*L0*zeta/(L1+L0*zeta);
    Theta1 = theta1*L1/(L1+L0*zeta);
end

kmp.PO_sb = [0 0 -d]';
kmp.RO_sb = Expm([0 0 phi+ga/180*pi]');

kmp.Psb_1b = calcSegP(Theta0,delta1,L0);
kmp.Rsb_1b = calcSegR(Theta0,delta1);

kmp.P1b_1e = calcSegP(Theta1,delta1,L1);
kmp.R1b_1e = calcSegR(Theta1,delta1);

kmp.P1e_2b = [0 0 Lr]';
kmp.R1e_2b = eye(3);

kmp.P2b_2e = calcSegP(theta2,delta2,L2);
kmp.R2b_2e = calcSegR(theta2,delta2);

kmp.P2e_g = [0 0 Lg]';
kmp.R2e_g = eye(3);


kmp.Psb_g_in_sb = kmp.Psb_1b+kmp.Rsb_1b*(kmp.P1b_1e+(kmp.R1b_1e*(kmp.P1e_2b+kmp.R1e_2b*(kmp.P2b_2e+kmp.R2b_2e*kmp.P2e_g))));
kmp.P1e_g_in_sb = kmp.Rsb_1b*kmp.R1b_1e*(kmp.P1e_2b+kmp.R1e_2b*(kmp.P2b_2e+kmp.R2b_2e*kmp.P2e_g));
kmp.P2e_g_in_2b = kmp.R2b_2e*kmp.P2e_g;

kmp.PO_g = kmp.PO_sb + kmp.RO_sb*(kmp.Psb_1b+kmp.Rsb_1b*(kmp.P1b_1e+kmp.R1b_1e*(kmp.P1e_2b+kmp.R1e_2b*(kmp.P2b_2e+kmp.R2b_2e*kmp.P2e_g))));
kmp.RO_g = kmp.RO_sb*kmp.Rsb_1b*kmp.R1b_1e*kmp.R1e_2b*kmp.R2b_2e*kmp.R2e_g;
end


function [p]=calcSegP(theta,delta,L)
if(abs(theta)<1e-7)
    p=[0 0 L]';
else
    p=L/theta*[cos(delta)*(1-cos(theta)) sin(delta)*(1-cos(theta)) sin(theta)]';
end
end


function [R]=calcSegR(theta,delta)
R = Expm([0 0 delta]')*Expm([0 theta 0]')*Expm([0 0 -delta]');
end

function [T]=Expm(u)
%simplified calculation for exponetial map (u in R3)
if(length(u)==3)
    theta=norm(u);
    if(theta == 0)
        R=eye(3);
    else
        un=u/theta;
        R=cos(theta)*eye(3)+(1-cos(theta))*un*un'+sin(theta)*S(un);
    end
    T=R;
elseif(length(u)==6)
    theta=norm(u(4:6));
    if(theta == 0)
        R=eye(3);
    else
        un=u(4:6)/theta;
        R=cos(theta)*eye(3)+(1-cos(theta))*un*un'+sin(theta)*S(un);
    end
    T=[R u(1:3);0 0 0 1];
else
    T=eye(4);
    disp('erroneous size of u!');
end
end


function [u]=Logm(R)
%simplified calculation for logarithmic map (u in R3)
theta = acos((R(1,1)+R(2,2)+R(3,3)-1)/2);
t=real(theta);
if t==0
    u=[0 0 0]';
else
    u=[R(3,2)-R(2,3) R(1,3)-R(3,1) R(2,1)-R(1,2)]'/2/sin(t)*t;
end

end


function T = S( p )
%this function gives the skew-symmetric matrix of the vector p

if(length(p)==3)
    T=[0 -p(3) p(2); p(3) 0 -p(1);-p(2) p(1) 0];
elseif(length(p)==6)
    R=[0 -p(6) p(5); p(6) 0 -p(4);-p(5) p(4) 0];
    T=[R p(1:3);zeros(1,4)];
end
end



