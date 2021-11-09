function [x0,x1] = setX0X1
    global path;
    fpath=['../conf/configurationData/arm',num2str(path),'/initPara.log'];
    beforeOpt=load(fpath);
    afterOpt=load('../conf/matlab/finalPara.log');
    
    
%     beforeOpt
%       L1      Lr     L2    Lg     gamma       K1      K2      E           Lstem     L0
%     x0=[79.2    30   19.4   2.5      0.1        0.5         1       50e9      381.5      1];
    x0=zeros(1,10);
    x0(1:7)=beforeOpt(1:7);
    x0(8)=beforeOpt(9);
    x0(9)=beforeOpt(8);
    
    x1=zeros(1,10);
    x1(1:7)=afterOpt(1:7);
    x1(8)=afterOpt(9);
    x1(9)=afterOpt(8);
% x0=[81.0654   29.4346   16.9999    6.3665    0.2924    0.96040    0.90019 -76.3906   382.013    -4.9994];
    
end