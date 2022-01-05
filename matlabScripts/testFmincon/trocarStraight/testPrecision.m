clear all;close all;clc;
%% test 59;
name='59_2';
[Tend_m,Psi]=getEnd(name);
Psi=Psi';


psi0=[0.0055    0.0005    0.0289    0.0020   -0.0000    0.0034]'*0;
% psi0=[-0.0051    0.0000    0.0692    0.0090    0.0101   -0.0141];
xh=[0.0575   -0.3201   -0.4780    0.0014   -0.0093   -0.0040]*0;
p=xh(1:3)/1000;
zyx=xh(4:6);
R=eul2rotm(zyx);
Ttt_trocar=[R,p';[0 0 0 1]];



block_size=max(size(Psi));
for i =block_size:-1:1
    if(Tend_m(3,4,i)==0||Psi(3,i)>pi/2.5)%||Psi(5,i)<pi/20||(Psi(3,i)+Psi(5,i)*(1-abs(Psi(6,i)-Psi(4,i))/pi))>pi/1
        Tend_m(:,:,i)=[];
        Psi(:,i)=[];
    end

end
block_size=max(size(Psi));
exitflag_kw=zeros(block_size,1);
d_block=zeros(block_size,1);
Tendc=zeros(4,4,block_size);
Tendn=zeros(4,4,block_size);
Tendn_no=zeros(4,4,block_size);
% L1x=-1.9744*1e-3-3.7e-3;
L1x=-5.5*1e-3;
dttc=9.9999*1e-3;
tic;
first_point=1;

for i = first_point:block_size

    if(Tend_m(3,4,i)==0)
        continue;
    end

    tem_psi=Psi(:,i);
    t=tem_psi(2);
    tem_psi(2)=tem_psi(1);
    tem_psi(1)=t;
    tem_psi=tem_psi.*[1 1e-3 1 1 1 1]'+psi0;
%     if(tem_psi(3)>pi/2.1)
%         continue;
%     end

    LL=tem_psi(2)+L1x;
    thetatheta=tem_psi(3);
    SP.psi.l=LL; % l>30
    SP.psi.phi=tem_psi(1);
    SP.psi.theta1=thetatheta;
    SP.psi.delta1=tem_psi(4);
    SP.psi.theta2=tem_psi(5);
    SP.psi.delta2=tem_psi(6);
    
    SP=setInitValV1(SP);
    
    
    theta1=SP.psi.theta1;
    xhmax=[10 40 10 10 ];
    xhmin=[-1e-9 -1e-9 -1e-9 0 ];
    options = optimoptions('fmincon','Algorithm','interior-point',"EnableFeasibilityMode",true); % 'Display','iter',,'PlotFcn','optimplotfirstorderopt','PlotFcn','optimplotfval'
    options.StepTolerance=1e-25;
    options.OptimalityTolerance=5e-6;
%     x=(xhmin+xhmax)/2;
%         x=rand(1,size(xhmin,2)).*(xhmax-xhmin)+xhmin;
    x=[0    0    2    9 ];
    count=0;
    exitflag=0;
    while(exitflag~=1)
        [xh,yh,exitflag] = fmincon('costFunc_u_Straight',x,[],[],[],[],xhmin,xhmax,'geth',options); %,options
        exitflag_kw(i)=exitflag;
        if(count>1000)
            fprintf('\n\ntem_psi: %f,%f,%f,%f,%f,%f\n\nexitflag:%d',tem_psi,exitflag);
            break;
        end
        count=count+1;
        x=rand(1,size(xhmin,2)).*(xhmax-xhmin)+xhmin;
    end
    SP=getSPV1;
    Tc=plotResult(0);
    Tn=plotResult_noClearance(0,dttc);
    Tn_no=plotResult_noClearance(0,0);
    Tm=Tend_m(:,:,i);
%     Tend_m(1:3,4,i)=Tend_m(1:3,4,i)+[0.62 0 0.11]'*1e-3;
    Tendc(:,:,i)=Ttt_trocar*Tc;
    Tendn(:,:,i)=Ttt_trocar*Tn;
    Tendn_no(:,:,i)=Ttt_trocar*Tn_no;
%     plotCoord(Tm);
%     axis([Tc(1,4)-10e-3 Tc(1,4)+10e-3 Tc(2,4)-10e-3 Tc(2,4)+10e-3 Tc(3,4)-10e-3 Tc(3,4)+10e-3])
% %     axis([-5e-3 +5e-3 -5e-3 +5e-3 -11e-3 1e-3])
%         cla;
    d_block(i)=SP.trocar.d;
end
%%
Terr_c=(Tend_m-Tendc);
Terr_n=(Tend_m-Tendn);
Terr_n_no=(Tend_m-Tendn_no);
dis_err_c=zeros(block_size,1);
dis_err_n=zeros(block_size,1);
dis_err_n_no=zeros(block_size,1);
T_dev=zeros(4,4,block_size);
X_dev=zeros(block_size,1);
Y_dev=zeros(block_size,1);
Z_dev=zeros(block_size,1);
z_dev=zeros(block_size,3);
a_dev=zeros(block_size,1);
for i =1 :block_size
    dis_err_c(i)=norm(Terr_c(1:3,4,i));
    dis_err_n(i)=norm(Terr_n(1:3,4,i));
    dis_err_n_no(i)=norm(Terr_n_no(1:3,4,i));
    T_dev(:,:,i)=inv(Tend_m(:,:,i))*Tendc(:,:,i);
    X_dev(i)=T_dev(1,4,i);
    Y_dev(i)=T_dev(2,4,i);
    Z_dev(i)=T_dev(3,4,i);
    z_dev(i,:)=reshape(T_dev(1:3,3,i),[1 3]);
    a_dev(i)=acosd(dot([0 0 1]',T_dev(1:3,3,i)));

end
if(min(exitflag_kw)>0&&max(exitflag_kw)<2)
    mean(dis_err_c)*1000
    mean(dis_err_n)*1000
    mean(dis_err_n_no)*1000
else
    fprintf("\n\nnon-convergence.\n\n");
end
toc;
%%
figure;hold on; axis equal;
a_dev_XZ=zeros(block_size,1);
a_dev_YZ=zeros(block_size,1);

for i =1:block_size
    plot3(T_dev(1,3,i),T_dev(2,3,i),T_dev(3,3,i),'g*');
    a_dev_XZ(i)=atand(T_dev(2,3,i)/T_dev(3,3,i));
    a_dev_YZ(i)=atand(T_dev(1,3,i)/T_dev(3,3,i));
    
end
figure;hold on;grid on;
plot(X_dev*1000,'r')
plot(Y_dev*1000,'g')
plot(Z_dev*1000,'b')
plot(dis_err_c*1000,'k')
title('在{target}下，测量结果的位置误差');xlabel('CF');ylabel('\iterror,mm');
legend('X dev','Y dev','Z dev','dis err');
figure;hold on;grid on;
plot(reshape(Tend_m(1,4,:)-Tendc(1,4,:),[1 block_size])*1000,'r')
plot(reshape(Tend_m(2,4,:)-Tendc(2,4,:),[1 block_size])*1000,'g')
plot(reshape(Tend_m(3,4,:)-Tendc(3,4,:),[1 block_size])*1000,'b')
plot(dis_err_c*1000,'k')
title('在{trocar}下，测量结果与目标位姿的位置误差');xlabel('CF');ylabel('\iterror,mm');
legend('X dev','Y dev','Z dev','dis err');
figure;hold on;grid on;
plot(a_dev_XZ,'c')
plot(a_dev_YZ,'k')
title('error');xlabel('CF');ylabel('\iterror,°');
legend('angle XZ','angle YZ');


%% 
tt=Psi(6,:)-Psi(4,:);
x=find(tt>pi);
tt(x)=tt(x)-pi*2;
x=find(tt<-pi);
tt(x)=tt(x)+pi*2;
figure;grid on;hold on;
plot(tt,dis_err_c*1000,'*');
plot(tt,a_dev,'*');
xlabel("\delta2-\delta1");
ylabel("error")
legend('与位置误差的关系','与姿态误差的关系');

ttmax=max(abs(tt));
theta12_delta12=(ttmax-tt)/ttmax.*Psi(5,:)*0.5+Psi(3,:);
figure;grid on;hold on;
plot(theta12_delta12,dis_err_c*1000,'*');
plot(theta12_delta12,a_dev,'*');
xlabel("\delta2-\delta1");
ylabel("error")
legend('与位置误差的关系','与姿态误差的关系');