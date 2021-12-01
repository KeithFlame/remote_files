clear,clc;
%% test 61;
name='61_2';
[Tend_m,Psi]=getEnd(name);


block_size=max(size(Psi));
for i =block_size:-1:1
    if(Tend_m(3,4,i)==0)
        Tend_m(:,:,i)=[];
        Psi(:,i)=[];
    end
end
block_size=max(size(Psi));
exitflag_kw=zeros(block_size,1);
Tendc=zeros(4,4,block_size);
Tendn=zeros(4,4,block_size);
L1x=-1.9790*1e-3;
dttc=9.4674*1e-3;
for i = 1:block_size

    if(Tend_m(3,4,i)==0)
        continue;
    end
    tem_psi=Psi(:,i);
    tem_psi=tem_psi.*[1e-3 1 1 1 1 1]';
    LL=tem_psi(1)+L1x;
    thetatheta=tem_psi(3);
    SP.psi.l=LL; % l>30
    SP.psi.phi=tem_psi(2);
    SP.psi.theta1=thetatheta;
    SP.psi.delta1=tem_psi(4);
    SP.psi.theta2=tem_psi(5);
    SP.psi.delta2=tem_psi(6);
    
    SP=setInitValV1(SP);
    
    
    theta1=SP.psi.theta1;
    xhmax=[10 40 10 10];
    xhmin=[-1e-9 -1e-9 -1e-9 0];
    options = optimoptions('fmincon','Algorithm','interior-point',"EnableFeasibilityMode",true); % 'Display','iter',,'PlotFcn','optimplotfirstorderopt','PlotFcn','optimplotfval'
    options.StepTolerance=1e-25;
    options.OptimalityTolerance=5e-6;
    x=(xhmin+xhmax)/2;
%         x=rand(1,size(xhmin,2)).*(xhmax-xhmin)+xhmin;
    count=0;
    exitflag=0;
    while(exitflag~=1)
        [xh,yh,exitflag] = fmincon('costFunc_u_Straight',x,[],[],[],[],xhmin,xhmax,'geth',options); %,options
        exitflag_kw(i)=exitflag;
        if(count>50)
            
            break;
        end
        count=count+1;
        x=rand(1,size(xhmin,2)).*(xhmax-xhmin)+xhmin;
    end
    SP=getSPV1;
    Tc=plotResult(0);
    Tn=plotResult_noClearance(0,dttc);
    Tendc(:,:,i)=Tc;
    Tendn(:,:,i)=Tn;
%     axis([Tc(1,4,i)-5e-3 Tc(1,4,i)+5e-3 Tc(2,4,i)-5e-3 Tc(2,4,i)+5e-3 Tc(3,4,i)-5e-3 Tc(3,4,i)+5e-3])
end
%%
Terr_c=(Tend_m-Tendc);
Terr_n=(Tend_m-Tendn);
dis_err_c=zeros(block_size,1);
dis_err_n=zeros(block_size,1);
for i =1 :block_size
    dis_err_c(i)=norm(Terr_c(1:3,4,i));
    dis_err_n(i)=norm(Terr_n(1:3,4,i));
end
if(sum(exitflag_kw)==block_size&&max(exitflag_kw)==1)
    mean(dis_err_c)*1000
    mean(dis_err_n)*1000
else
    fprintf("\n\nnon-convergence.\n\n");
end