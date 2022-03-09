function [T0,Te,config,flag,S]=getInitValue(psi)
%
%
%
    
    [config,flag]=getInnerTrocarPara(psi);
    [T0,Te,S]=getInitT(config);
end