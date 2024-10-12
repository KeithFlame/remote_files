function psi_out=resetPsi_Phi1(psi_in)
    t=psi_in(1);
    psi_in(1)=psi_in(2);
    psi_in(2)=t;
    psi_out=psi_in;
    psi_out(2:6)=psi_out(2:6)/pi*180;
end