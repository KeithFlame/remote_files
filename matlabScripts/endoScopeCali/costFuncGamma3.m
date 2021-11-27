function [f]=costFuncGamma3(gamma3)
    L1=18;Lr=8;L2=30;Lg=19;
    [T,endoPsi]=getTandConfigV2;
    Tgamma3=[cos(gamma3) -sin(gamma3) 0 0;sin(gamma3) cos(gamma3) 0 0; 0 0 1 0; 0 0 0 1];
    config=zeros(1,10);
    cter=ones(1,size(endoPsi,1))*100;
    for i = 1:size(endoPsi,1)
        tempT=T(:,:,i)*Tgamma3;
        tempR=tempT(1:3,1:3);
        config(1)=endoPsi(i,1);config(2)=endoPsi(i,2);config(3)=endoPsi(i,3);
        config(4)=L1;config(5)=endoPsi(i,4);config(6)=Lr;config(7)=endoPsi(i,5);config(8)=L2;
        config(9)=endoPsi(i,6);config(10)=Lg;
        [Tt,~,~,~,~,~,~,~,~,~,~]=FKfunc(config);
        Rt=Tt(1:3,1:3);
        axang=rotm2axang(Rt'*tempR);
        cter(i)=axang(4)*180/pi;
    end
    f=norm(cter);
        
        
