function psi = Curvature2Psi_keith(u,SL,l)
% this is afunction to get configuration variables of a 2-seg continuum 
% manipulator form a known curvature and several structural parameters.
%
% Author: Keith W.
% Ver. 1.0
% Date: 15.02.2022

L1=SL(1)*1e-3;
L2=SL(3)*1e-3;
zeta=SL(5);
psi=zeros(1,6);


u_=u;
if(l<L1)
    L1=l;
    psi(3)=L1*norm(u_(1:3));
else
    psi(3)=(L1+zeta*(l*1e-3-L1))*norm(u_(1:3));
end
psi(4)=-pi/2+atan2(u_(2),u_(1));
psi(5)=L2*norm(u_(4:6));
psi(6)=-pi/2+atan2(u_(5),u_(4));

end


