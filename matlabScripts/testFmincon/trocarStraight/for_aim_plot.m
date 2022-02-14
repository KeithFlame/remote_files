clear all;close all;clc;

% Psi=0:5:90;
L0=(0:5:90)+63;
block_size=max(size(L0));
exitflag_kw=zeros(block_size,1);
d_block=zeros(block_size,1);
dis_block=zeros(block_size,1);
Tendc=zeros(4,4,block_size);
Tendn=zeros(4,4,block_size);
Tendn_no=zeros(4,4,block_size);
% L1x=-1.9744*1e-3-3.7e-3;
L1x=0*1e-3;
dttc=0*1e-3;
tic;
first_point=1;

for i = first_point:block_size

    LL=L0(i)/1000;
%     thetatheta=Psi(i)/180*pi;
    thetatheta=pi/2;
    SP.psi.l=LL; 
    SP.psi.phi=0;
    SP.psi.theta1=thetatheta;
    SP.psi.delta1=pi/2;
    SP.psi.theta2=0;
    SP.psi.delta2=0;
    
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
    Tendc(:,:,i)=Tc;
    Tendn(:,:,i)=Tn;
    Tendn_no(:,:,i)=Tn_no;
    d_block(i)=SP.trocar.d;
    dis_block(i)=norm(Tc(1:3,4)-Tn_no(1:3,4));
end
%%
plot(L0,dis_block)
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