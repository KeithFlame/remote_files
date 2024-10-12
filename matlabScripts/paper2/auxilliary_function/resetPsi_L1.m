function psi_out=resetPsi_L1(psi_in)
    psi_in(2:6)=psi_in(2:6)*pi/180;
    t=psi_in(1);
    psi_in(1)=psi_in(2);
    psi_in(2)=t;
    psi_out=psi_in;
end

