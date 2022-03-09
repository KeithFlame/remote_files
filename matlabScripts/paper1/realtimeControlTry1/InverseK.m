function [Psi,S]=InverseK(T)
%
%
%
%
    psi=getPsi(T);
    [T0,Te,config,~,~]=getInitValue(psi);

    errT=Te\T;
    axang=rotm2axang(errT(1:3,1:3));
    errP=norm(errT(1:3,4));
    err=[axang(4)*180/pi errP*1000];
    cter=0.1;
    i=0;
%     T_=T;
%     T_=T0\T;
%     psi=getPsi(T_);
    Tt=T;
    Tn=T;
    while(max(err)>cter)
        if(psi(3)<pi/180)
            
            break;
        end
        T_=T0\Tn;
        i=i+1;
        psi=getPsi(T_);
        psi(3)=sum(config(5:6))+psi(3);
%         psi(5)=psi(5)-0.01*sum(config(5:6));
%         psi(2)=psi(2)+0.01*(sum(config(1:2))-config(9));
        [~,Te,~,~,~]=getInitValue(psi);
        
        errT=Te\Tt;
        axang=rotm2axang(errT(1:3,1:3));
        errP=norm(errT(1:3,4));
        err=[axang(4)*180/pi errP*1000];
        Tn=Tt*errT;
        psi0=getPsi(Tn);
        [T0,~,config,~,~]=getInitValue(psi0);
    end
    Psi=psi;
    [~,~,~,~,S]=getInitValue(psi);
end

