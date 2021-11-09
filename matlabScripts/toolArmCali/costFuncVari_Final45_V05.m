function f = costFuncVari_Final45_V05(x)
%

% Date£º13.43 09.01.2021
global Tst2tc;
global Tst2tip_ini;
global Tst2tip_ini2;
global endoPsi_ini;
global OptSerial;
global optTimes;
global residualVal;
global isSimplified;
global TLm2marker;
global gamma1;
xN=zeros(1,10);
[x0,~]=setX0X1;
x0N=x0;
    xN(1)=99.2;
    xN(2)=x(1);
    xN(3)=19.4;
    xN(4)=x(2);
    xN(5)=0.1;
    xN(6)=x(3);  %K1
    xN(7)=x(4)/10;
    if(isSimplified==1)
        xN(8)=391.5-x(1);
    else
        xN(8)=411.5-x(1);
    end
    xN(9)=gamma1;%x(8)/10;%
    
    xN(10)=x(6);
    
    
    L1x=x(5);
    dttc=x(6);
    Ttc2ttc=eye(4);
    Ttc2ttc(3,4)=dttc;
    Ttc2st=inv(Tst2tc*Ttc2ttc);
    TLm2marker_used=TLm2marker;

        
    beta=x(7);

   TLg2Lm=eye(4);TLm2marker1Position=eye(4);Tmarker1Position2YZ=eye(4);TYZ2Z=eye(4);
    x_rot_ang=acos(dot([0 0 1]',TLm2marker_used(1:3,3)));
    TLg2Lm(1:3,1:3)=[cos(xN(9)) -sin(xN(9)) 0 ;sin(xN(9)) cos(xN(9)) 0 ;0 0 1]';
    
    TLm2marker1Position(1:3,4)=TLm2marker_used(1:3,4);
    Tmarker1Position2YZ(1:3,1:3)=[cos(beta) -sin(beta) 0 ;sin(beta) cos(beta) 0 ;0 0 1];
    TYZ2Z(1:3,1:3)=[1 0 0;0 cos(x_rot_ang) -sin(x_rot_ang);0 sin(x_rot_ang) cos(x_rot_ang) ];
    
se=[  4:5:size(endoPsi_ini,1) 5:5:size(endoPsi_ini,1) ];
    Tst2tip=Tst2tip_ini(:,:,se);
    Tst2tip2=Tst2tip_ini2(:,:,se);
    endoPsi=endoPsi_ini(se,:);
    
    
    dp=zeros(size(Tst2tip,3),1);
    dp1=zeros(size(Tst2tip,3),3);
    dAngle=zeros(size(Tst2tip,3),1);
    
    enPsi=zeros(size(endoPsi_ini,1),6);
    
    iter=0;
    for i =1:size(Tst2tip,3)     
        
        temEndoPsi=endoPsi(i,:);

        endoU=fromPsi2Curvature(temEndoPsi,x0N);
        
        endoQ=fromCurvature2Movitation(endoU,x0N,temEndoPsi);
        
        temEndoPsi(2)=temEndoPsi(2)+L1x-dttc;
        
        endoU_=fromMovitation2Curvature(endoQ,xN,temEndoPsi);
        
        endoPsi_=fromCurvature2Psi(endoU_,xN,temEndoPsi);
        
        endoPsi_(:,1:2)=temEndoPsi(:,1:2);
       
        [Ttrocar2Lg,~,~,~,~,~,~,~,~,~,~]=fromPsi2Config(endoPsi_,xN);
        
        enPsi(i,:)=endoPsi_;
        
        Ttar=Ttrocar2Lg*TLg2Lm*TLm2marker1Position*Tmarker1Position2YZ*TYZ2Z;
        
        
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
    dp_sorted=sort(dp);
    dAngle_sorted=sort(dAngle);
%     t3=max([sum(dp_sorted(end-5:end)), 10, sum(dAngle_sorted(end-5:end)) ]);


    f=t3;
    
    optTimes=optTimes+1;
    OptSerial(optTimes,:)=x;
    residualVal(optTimes)=f;

end



