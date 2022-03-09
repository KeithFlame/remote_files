function KP=setPsi_U(psi,us,u1)
%
%
%
    persistent KP0;
    if(nargin==0)
        KP=KP0;
        return;
    end
    
    KP0.psi=psi;
    KP0.u=[us,u1];
end
    