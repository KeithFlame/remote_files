function SPC = getStructureParaforCDR_keith(is_refresh)
% this is function to obtain structure parameters by the files
% "strcture_parameter_CDR" , which is located as the current files.
% 
% Author Keith W.
% Ver. 1.0
% Date 04.13.2022

if(nargin == 0)
    is_refresh = 0;
end

persistent SP0;
if(is_refresh||isempty(SP0))
    E = 62e9;        %杨氏模量  
    G = 23.3082e9;   %截切模量  
    r = 0.0012/2;     %横截面半径
    
    
    %合成变量
    A = pi * r^2;   %横截面面积
    I = pi * r^4 / 4; %二次面积矩
    Kse = 5 * diag([G*A E*A]);
    Kbt = 5 * E*I;
    
    % alpha beta phi Li Di r
    size_para = load('./structure_parameters_CDR/size_para.raw');
    size_para(1:3) = size_para(1:3)*pi/180;
    size_para(4:6) = size_para(4:6)/1000;
    SP0.size_para = size_para;
    SP0.Kse = Kse;
    SP0.Kbt = Kbt;

    % gravity for multi-lumen tube moving platform external load
    external_load = load('./structure_parameters_CDR/gravity_para.raw');

    gravity_para = [2.4059 0.7448 external_load];
    SP0.gravity_para = gravity_para;
end

SPC=SP0;

end