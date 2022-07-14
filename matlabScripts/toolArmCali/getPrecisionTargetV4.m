function [finalValue2, validateConfig]=getPrecisionTargetV4
global LS;
global Tst2tc;
global precisionTarget;
global iter;
global XH1_7;
precisionTarget=[0 0 102.5 0 0 0
                34 34 136.5 0 0 -pi/4
                -34 34 136.5 0 0 pi/4
                -34 -34 68.5 pi/4 0 -pi/4
                34 -34 68.5 pi/4 0 pi/4
                
  ];

finalValue=zeros(11,8);finalValue(:,1)=40;
validateConfig=zeros(4,4,5);
xxxx=0;
if(xxxx==0)
    xN = [80 30 20 15 0.1964 6.9978 0.7177...
        376.7004 -0.7854 -7.9171 2.2758 -0.7562 ];
%     xN=load('../conf/matlab/finalPara.log');
%     xOpt1=load('../conf/matlab/initPara.log');
else
    xN=load('../conf/matlab/finalPara.log');
end

gamma1=xN(9);
dttc=xN(10);
Ttc2ttc=eye(4);
Ttc2ttc(3,4)=dttc;
% Tst2ttc=Tst2tc*Ttc2ttc; 
L1x=0;
LS=xN;
% 
gripperLength=15;
% 
% t=load('../conf/matlab/Tmarker2Lm.log');
% TLm2marker=inv(reshape(t,[4  4]));
% TLm2marker_used=TLm2marker;
% beta=xN(11);
% T2e2Lg=eye(4);TLg2Lm=eye(4);TLm2marker1Position=eye(4);Tmarker1Position2YZ=eye(4);TYZ2Z=eye(4);
% T2e2Lg(3,4)=xN(4);
% x_rot_ang=acos(dot([0 0 1]',TLm2marker_used(1:3,3)));
% TLg2Lm(1:3,1:3)=[cos(xN(9)) -sin(xN(9)) 0 ;sin(xN(9)) cos(xN(9)) 0 ;0 0 1]';
% 
% TLm2marker1Position(1:3,4)=TLm2marker_used(1:3,4);
% Tmarker1Position2YZ(1:3,1:3)=[cos(beta) -sin(beta) 0 ;sin(beta) cos(beta) 0 ;0 0 1];
% TYZ2Z(1:3,1:3)=[1 0 0;0 cos(x_rot_ang) -sin(x_rot_ang);0 sin(x_rot_ang) cos(x_rot_ang) ];
%  
% T2e2testpoint=eye(4);T2e2testpoint(3,4)=15;
% Tall=inv(inv(T2e2Lg*TLg2Lm*TLm2marker1Position*Tmarker1Position2YZ*TYZ2Z)*T2e2testpoint);

Tall =eye(4);

for iter =1:size(precisionTarget,1)
    % 优化初值

    R=rotTest(precisionTarget(iter,4),precisionTarget(iter,5),precisionTarget(iter,6));
    P=precisionTarget(iter,1:3)';
    P(3)=P(3)-dttc;
    T=[R,P;[0 0 0 1]];
   
    if(P(3)>115-dttc)
        config=invKine(T,LS(1),LS(2),LS(3),gripperLength);
        config(2)=config(4)+config(2);
    else
        config=IKv5(T,LS(2),LS(3),gripperLength);
        config(2)=config(4)+config(2); 

    end
    config(5)=config(5)+config(1);
    config(9)=config(9)+config(1);
    config(1)=0;
%% calc 实际的2,3构型位置

if(config(2)>LS(1))
    options = optimoptions('fmincon','Algorithm','interior-point'); % 
    options.StepTolerance=1e-25;
    options.OptimalityTolerance=5e-6;
    xinit=[config(1) config(2)/100 config(3) config(5) config(7) config(9)];
    [x1,y1,exitFlag]=fmincon('getTrueC4',xinit,[],[],[],[],[],[],[],options);
    config(1)=0;
    config(2)=x1(2)*100;
    config(3)=x1(3);
    config(5)=x1(1)+x1(4);
    config(7)=x1(5);
    config(9)=x1(1)+x1(6);
end
%%   
    %计算该构型下marker位置
    
        xN=LS;
        xN(9)=0;
        xN(4)=gripperLength;
        xP=[config(1) config(2) config(3) config(5) config(7) config(9)];
        [Tcm,~,~,~,~,~,~,~,~,~,~]=fromPsi2Config(xP,xN);
%         config(end)=LS(4);
        [Tcc,~,~,~,~,~,~,~,~,~,~]=FKfunc2(config);
        
        if(config(2)==config(4))
            config(2)=config(2)-L1x+dttc;
        else
            config(2)=config(2)-L1x+dttc;
        end
        
        Tc=Ttc2ttc*Tcm;
        x=[config(1) config(2) config(3) config(5) config(7) config(9)];
        
        x(2)=x(2)*pi/180;
        temx2=x(2);
        x(2)=x(1);
        x(1)=temx2;
        x=x*180/pi;
%         x(1) = x(1);
        finalValue(iter*2,1:6)=x;
        if(finalValue(iter*2, 3)==0)
             finalValue(iter*2,[4 6])= finalValue(iter*2,[4 6]);
        else
            finalValue(iter*2,[4 6])= finalValue(iter*2,[4 6])-gamma1*180/pi;
        end
        finalValue(iter*2,8)=10;
        finalValue(iter*2-1,8)=4;
        if(iter*2-2>0)
            finalValue(iter*2-1,1)=finalValue(iter*2-2,1);
        end
        validateConfig(:,:,iter)=Tc*Tall;
end
finalValue(3,1)=finalValue(4,1);
finalValue(end,1)=finalValue(end-1,1);
finalValue=[finalValue(1:7,:);finalValue(9,:);finalValue(8:end,:)];
finalValue(end,8)=4;
finalValue2=[finalValue;
    finalValue(2:12,:);
    finalValue(2:12,:);
    finalValue(2:12,:);
    finalValue(2:12,:);
    ];
finalValue2=[finalValue2;[40 0 0 0 0 0 0 4]];

end
