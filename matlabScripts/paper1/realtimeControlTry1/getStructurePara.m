function trocar=getStructurePara
%
%
%
    persistent tro;
    if(isempty(tro))
        tro.c=0.5e-3;
        tro.zeta=0.15;
    end
    trocar=tro;
end