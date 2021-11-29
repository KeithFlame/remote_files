function f = costFuncVari_Final45_V0S_1110(x)
%

% Date：13.43 09.01.2021
global Tst2tip_ini;
global endoPsi_ini;
global qa_actual;
global OptSerial;
global optTimes;
global residualVal;
global xN_init;
xN=zeros(1,10);

xN(1)=x(8)*10; %99.2;%
xN(2)=x(1);
xN(3)=19.4;
xN(4)=x(2);
xN(5)=x(9)/10; %0.1;%
xN(6)=x(3);  
xN(7)=x(4)/10;
xN(8)=391.5-x(1);
xN(9)=0*x(5)/10;
xN(10)=x(7);


L1x=x(6);
dttc=x(7);
Ttc2ttc=eye(4);
Ttc2ttc(3,4)=dttc;
Ttc2st=inv(Ttc2ttc);
    


%se=1:size(endoPsi_ini,1)-34;%-30;

block_size=size(endoPsi_ini,1);

se=zeros(block_size,1);
for i=1:block_size
    if(Tst2tip_ini(3,4,i)~=0) %(endoPsi_ini(i,2)+L1x-dttc)<xN(1)&&
        se(i)=i;
    end
end
se(all(se==0,2),:)=[];

Tst2tip=Tst2tip_ini(:,:,se);
endoPsi=endoPsi_ini(se,:);
Qa_used=qa_actual(se,:);

dp=zeros(size(Tst2tip,3),1);
dp1=zeros(size(Tst2tip,3),3);
dAngle=zeros(size(Tst2tip,3),1);

enPsi=zeros(size(endoPsi_ini,1),6);

% x0N=setX0X1;
x0N=xN_init;
t=x0N(8);
x0N(8)=x0N(9);
x0N(9)=t;
x0N=[100 10 20 15 0.1 5 0.7 -0.7854 381.5 -10 -2];
iter=0;
for i =1:size(Tst2tip,3)     
    
    temEndoPsi=endoPsi(i,:);

    endoU=fromPsi2Curvature(temEndoPsi,x0N);
    
    endoQ1=fromCurvature2Movitation(endoU,x0N,temEndoPsi);
    endoQ=Qa_used(i,:);
%         max(abs(endoQ1([1:4 end-3:end])-endoQ([1:4 end-3:end])))
    temEndoPsi(2)=temEndoPsi(2)+L1x-dttc;
    
    endoU_=fromMovitation2Curvature(endoQ,xN,temEndoPsi);
    
    endoPsi_=fromCurvature2Psi(endoU_,xN,temEndoPsi);
    
    endoPsi_(:,1:2)=temEndoPsi(:,1:2);
%         endoPsi_=endoPsi_.*[1 1 1 -1 1 -1 ];
    [T,~,~,~,~,~,~,~,~,~,~]=fromPsi2Config(endoPsi_,xN);
    enPsi(i,:)=endoPsi_;
    Ttar=T;
    Tst2tipi=Tst2tip(:,:,i);    
    Tmea=Ttc2st*Tst2tipi;
    dp(i)=norm(Tmea(1:3,4)-Ttar(1:3,4));
    dp1(i,:)=Tmea(1:3,4)'-Ttar(1:3,4)';
    
    z=Tmea(1:3,3)/norm(Tmea(1:3,3));
    
    dAn2=acos(dot(Ttar(1:3,3),z));
    
    dAngle(i)=abs(dAn2)*180/pi;
    
    iter=iter+1;

end

% t=norm(sum(dp1));
t2=sum(dp)+sum(dAngle);
t3=t2/iter;


f=t3;

optTimes=optTimes+1;
OptSerial(optTimes,:)=x;
residualVal(optTimes)=f;
end