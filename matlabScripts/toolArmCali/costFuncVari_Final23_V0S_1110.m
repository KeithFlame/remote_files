function f = costFuncVari_Final23_V0S_1110(x)
%

% Date：13.43 09.01.2021

global Tst2tip_ini;
global Tst2tip_ini2;
global endoPsi_ini;
global qa_actual;
% global OptSerial;
% global optTimes;
global isSimplified;
global XH1_7;
xOptimalized=XH1_7;
xN=zeros(1,10);
    xN(1)=x(1)*10;
    xN(2)=xOptimalized(1);
    xN(3)=19.4;
    xN(4)=xOptimalized(2);
    xN(5)=x(2)/10;
    xN(6)=xOptimalized(3);
    xN(7)=xOptimalized(4)/10;
    if(isSimplified==1)
        xN(8)=391.5+99.2-xOptimalized(1)-x(1)*10;
    else
        xN(8)=411.5+99.2-xOptimalized(1)-x(1)*10;
    end
    xN(9)=xOptimalized(5)/10;
    xN(10)=xOptimalized(7);
    [x0,~]=setX0X1;
    
    L1x=xOptimalized(6);
    dttc=xOptimalized(7);
    Ttc2ttc=eye(4);
    Ttc2ttc(3,4)=dttc;
    Ttc2st=inv(Ttc2ttc);  
        
    x0N=x0;

    %se=size(endoPsi_ini,1)-33:size(endoPsi_ini,1);
    load('0913\se_c4.mat');
    Tst2tip=Tst2tip_ini(:,:,se);
    Tst2tip2=Tst2tip_ini2(:,:,se);
    endoPsi=endoPsi_ini(se,:);
    Qa_used=qa_actual(se,:);
    
    dp=zeros(size(Tst2tip,3),1);
    dp1=zeros(size(Tst2tip,3),3);
    dAngle=zeros(size(Tst2tip,3),1);
    
    enPsi=zeros(size(endoPsi_ini,1),6);
    

    iter=0;
%     x0N=[98.7353    9.9905   19.4000   20.4665    0.3282 4.8918    0.9991   -0.6917   383.8595  -7.9929];

    for i = 1:size(Tst2tip,3)     
        
        temEndoPsi=endoPsi(i,:);
        
        endoU=fromPsi2Curvature(temEndoPsi,x0N);
        
        endoQ=fromCurvature2Movitation(endoU,x0N,temEndoPsi);
        endoQ=Qa_used(i,:);
        
        temEndoPsi(2)=temEndoPsi(2)+L1x-dttc;
        
        endoU_=fromMovitation2Curvature(endoQ,xN,temEndoPsi);
        
        endoPsi_=fromCurvature2Psi(endoU_,xN,temEndoPsi);
        
        endoPsi_(:,1:2)=temEndoPsi(:,1:2);
        
        [T,~,~,~,~,~,~,~,~,~,~]=fromPsi2Config(endoPsi_,xN);
        enPsi(i,:)=endoPsi_;
        Ttar=T;
        Tst2tipi=Tst2tip(:,:,i);
        Tst2tip2i=Tst2tip2(:,:,i);

        Tmea=Ttc2st*Tst2tipi;
        Tmea2=Ttc2st*Tst2tip2i;
        dp(i)=norm(Tmea(1:3,4)-Ttar(1:3,4));
        dp1(i,:)=Tmea(1:3,4)'-Ttar(1:3,4)';
        
        z=(Tmea2(1:3,4)-Tmea(1:3,4))/norm(Tmea2(1:3,4)-Tmea(1:3,4));
        
        dAn=rotm2axang(Ttar(1:3,1:3)'*Tmea(1:3,1:3));
        dAn1=acos(dot(Ttar(1:3,3),Tmea(1:3,3)));
        dAn2=acos(dot(Ttar(1:3,3),z));

        dAngle(i)=abs(dAn2)*180/pi;

        iter=iter+1;
    end
    
    t=norm(sum(dp1));
    t2=sum(dp)+sum(dAngle);
    t3=t2/iter;
    f=t3;

end




