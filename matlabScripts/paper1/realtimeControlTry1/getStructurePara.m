function trocar=getStructurePara
%
%
%
    persistent tro;
    if(isempty(tro))
        tro.c=0.5e-3;
        tro.zeta=0.15;
        tro.LS=[0.1038 0.00957 0.0194 0.015];
        tro.Lstem=0.3815;
        tro.K1=0.19;
        
    end
    trocar=tro;
end