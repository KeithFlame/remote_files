function structure_para_res=getStructurePara
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.22.2021
% Ver. 1.0
% output1: structure_para
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
persistent structure_para;
if(~isempty(structure_para))
    structure_para=zeros(1,12);
    fres=fopen("finalPara.log");
    if(fres==-1)
        L1=100;
        Lr=10;
        L2=19.4;
        Lg=15;
        zeta=0.1;
        K1=5;
        K2=0.6;
        gamma_a=-0.7854;
        Lstem=381.5;
        L1x=-2;
        d=-10;
        flag=0;
        structure_para=[L1 Lr L2 Lg zeta K1 K2 gamma_a Lstem L1x d flag];
    else
        tline=fgetl(fres);
        tlines=strsplit(tline, ' ');
        for i = 1:size(tlines,2)
            structure_para(i)=str2double(tlines(i));
        end
    
    
    end
    structure_para_res=structure_para;
end

end


