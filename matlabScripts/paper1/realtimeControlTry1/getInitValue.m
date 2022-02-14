function [T0,l,flag,S]=getInitValue(psi,LS)
%
%
%
    [config,flag]=getInnerTrocarPara(psi,LS);
    [T0,S]=getInitT(config);
    if(config(2)~=0)
        l=psi(2)-sum(LS(1:3))+d-config(1:3);
    else
        l=psi(2)-sum(LS(2:3))+d-config(1:3);
    end
end