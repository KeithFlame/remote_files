function cter =getCurvatureNoClearance(psi)
%
%
%
    
    tro=getStructurePara;
    LS=tro.LS;
    L1=LS(1);
    Lstem=tro.Lstem;
    zeta=tro.zeta;

    theta1s=psi(3);
    theta1=L1/(zeta*Lstem+L1)*theta1s;
    thetas= theta1s-theta1;
    u1=theta1/L1;
    us=thetas/Lstem;
    
    setPsi_U(psi,us, u1);
    if(theta1<pi/360)
        cter=1;
    else
        cter=0;
    end
end
