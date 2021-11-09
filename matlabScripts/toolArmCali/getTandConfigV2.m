function [T1,T2,Config,Tst2tc,TLm2marker]=getTandConfigV2
global path;
% path=load('../conf/configurationData/filePath.log');
fpathM1=['../conf/configurationData/arm',num2str(path),'/optimalConfigData_C1.log'];
fpathM2=['../conf/configurationData/arm',num2str(path),'/optimalConfigData_C2.log'];
fpathCF=['../conf/configurationData/arm',num2str(path),'/optiPoseSimplified.log'];
fpathTst2tc=['../conf/configurationData/arm',num2str(path),'/Tst2tc.log'];
fpathTmarker2Lm=['../conf/configurationData/arm',num2str(path),'/Tmarker2Lm.log'];

M=load(fpathM1);
M2=load(fpathM2);
CF=load(fpathCF);

t=load(fpathTst2tc);
Tst2tc=reshape(t,[4  4]);
t=load(fpathTmarker2Lm);
TLm2marker=inv(reshape(t,[4  4]));

Cf=zeros(3,6);j=1;
for i =1:size(CF,1)
    if(CF(i,8)>8)
        Cf(j,:)=CF(i,1:6);
        j=j+1;
    end
end
tem=Cf(:,1);
Cf(:,1)=Cf(:,2);
Cf=Cf*pi/180;
Cf(:,2)=tem;


T1=zeros(4,4,size(M,1));
T2=zeros(4,4,size(M2,1));
for i = 1:size(M,1)
    
    R1=quat2rotm(M(i,4:7));
    P1=M(i,1:3)';
    T1(1:3,1:3,i)=R1;
    T1(1:3,4,i)=P1;
    T1(4,4,i)=1;
    
    
    R2=quat2rotm(M2(i,4:7));
    P2=M2(i,1:3)';
    T2(1:3,1:3,i)=R2;
    T2(1:3,4,i)=P2;
    T2(4,4,i)=1;
end

Config=Cf;

% Tst2tc(1,4)=0.6+Tst2tc(1,4);Tst2tc(3,4)=1+Tst2tc(3,4);

end