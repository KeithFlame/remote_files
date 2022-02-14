function config = getInnerTrocarPara(psi,LS)
%
%
%
    [us,u1]=getCurvatureNoClearance(psi,LS);
    if(psi(2)>LS(1))
        config=zeros(6,1);
        config=getBaseStemInTrocar(psi,LS,us,u1);
    end

end




function config=getBaseStemInTrocar(psi,LS,us,u1,config)
    trocar = getStructurePara;
    c=trocar.c;
    zeta=trocar.zeta;
    l=psi(2);
    theta1=psi(3);
    thetasi=config(3);
    persistent thetasi0;
    thetasi0=0;

    
    sTsi=sin(thetasi);
    cTsi=cos(thetasi);
    
    d=c*sTsi./(1-cTsi);
    lsi=d.*thetasi./sTsi;
    lso=l-sum(LS(1:3))+d-lsi;
    thetaso=(theta1-thetasi).*lso*zeta./(lso*zeta+L1);
    theta1o=theta1-thetaso-thetasi;
    f=0.5*k1*L1*(theta1o/L1-u1)^2+0.5*ks*(lsi*(thetasi/lsi-us)^2+lso*(thetaso/lso-us)^2+(Lstem-lsi-lso)*us^2);


end
