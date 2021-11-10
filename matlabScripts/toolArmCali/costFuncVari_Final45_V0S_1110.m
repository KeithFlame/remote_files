function f = costFuncVari_Final45_V0S_1110(x)
%

% Date：13.43 09.01.2021
global Tst2tip_ini;
global endoPsi_ini;
global qa_actual;
global OptSerial;
global optTimes;
global residualVal;
xN=zeros(1,10);
% [x0,~]=setX0X1;
% x0N=x0;
    xN(1)=99.2;
    xN(2)=x(1);
    xN(3)=19.4;
    xN(4)=x(2);
    xN(5)=0.1;
    xN(6)=x(3);  %K1
    xN(7)=x(4)/10;
    xN(8)=391.5-x(1);
    xN(9)=x(5)/10;
    xN(10)=x(7);
    
    
    L1x=x(6);
    dttc=x(7);
    Ttc2ttc=eye(4);
    Ttc2ttc(3,4)=dttc;
    Ttc2st=inv(Ttc2ttc);    
        
    

    %se=1:size(endoPsi_ini,1)-34;%-30;
    se=[1:60 62 63 64];
%     load('0913\se_c3.mat');
    Tst2tip=Tst2tip_ini(:,:,se);
    endoPsi=endoPsi_ini(se,:);
    Qa_used=qa_actual(se,:);
    
    dp=zeros(size(Tst2tip,3),1);
    dp1=zeros(size(Tst2tip,3),3);
    dAngle=zeros(size(Tst2tip,3),1);
    
    enPsi=zeros(size(endoPsi_ini,1),6);
    
% x0N=[98.7353    9.9905   19.4000   20.4665    0.3282 4.8918    0.9991   -0.6917   383.8595  -7.9929];
% x0N=[97.9382    9.8953  19.4000 19.5123     0.1311   4.4183    0.9636    -0.7454   383.9463  -9.8943];
    iter=0;
    for i =1:size(Tst2tip,3)     
        if(Tst2tip_ini(3,4,i)~=0)
            temEndoPsi=endoPsi(i,:);
    %         temEndoPsi=[0, 10, 0.2, -0.5236, 1.3708, 0.9774];
    %         endoU=fromPsi2Curvature(temEndoPsi,x0N);
            
    %         endoQ=fromCurvature2Movitation(endoU,x0N,temEndoPsi);
            endoQ=Qa_used(i,:);
            
            temEndoPsi(2)=temEndoPsi(2)+L1x-dttc;
            
            endoU_=fromMovitation2Curvature(endoQ,xN,temEndoPsi);
            
            endoPsi_=fromCurvature2Psi(endoU_,xN,temEndoPsi);
            
            endoPsi_(:,1:2)=temEndoPsi(:,1:2);
            
            [T,~,~,~,~,~,~,~,~,~,~]=fromPsi2Config(endoPsi_,xN);
            enPsi(i,:)=endoPsi_;
            Ttar=T;
            Tst2tipi=Tst2tip(:,:,i);    
            Tmea=Ttc2st*Tst2tipi;
            dp(i)=norm(Tmea(1:3,4)-Ttar(1:3,4));
            dp1(i,:)=Tmea(1:3,4)'-Ttar(1:3,4)';
            
            z=Tmea(1:3,4)/norm(Tmea(1:3,4));
            
    %         dAn=rotm2axang(Ttar(1:3,1:3)'*Tmea(1:3,1:3));
    %         dAn1=acos(dot(Ttar(1:3,3),Tmea(1:3,3)));
            dAn2=acos(dot(Ttar(1:3,3),z));
    
            dAngle(i)=abs(dAn2)*180/pi;
    
            iter=iter+1;
        end
    end
    
%     t=norm(sum(dp1));
    t2=sum(dp)+sum(dAngle);
    t3=t2/iter;


    f=t3;
    
    optTimes=optTimes+1;
    OptSerial(optTimes,:)=x;
    residualVal(optTimes)=f;
end