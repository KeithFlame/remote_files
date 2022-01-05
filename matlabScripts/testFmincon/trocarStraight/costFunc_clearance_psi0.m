function f=costFunc_clearance_psi0(x)
%% test 59;
name='59_2';
[Tend_m,Psi]=getEnd(name);
Psi=Psi';
x=x./[10 0.5 10 1 10 1]*pi;
psi0=x;
xh=[0.0575   -0.3201   -0.4780    0.0014   -0.0093   -0.0040]*0;
p=xh(1:3)/1000;
zyx=xh(4:6);
R=eul2rotm(zyx);
Ttt_trocar=[R,p';[0 0 0 1]];
block_size=max(size(Psi));
for i =block_size:-1:1
    if(Tend_m(3,4,i)==0||(Psi(3,i)+Psi(5,i)*(1-abs(Psi(6,i)-Psi(4,i))/pi))>pi/1.7||Psi(3,i)>pi/2.5||Psi(5,i)<pi/20)
        Tend_m(:,:,i)=[];
        Psi(:,i)=[];
    end
end
block_size=max(size(Psi));
exitflag_kw=zeros(block_size,1);
Tendc=zeros(4,4,block_size);
L1x=-1.9744*1e-3-3.7e-3;
parfor i = 1:block_size

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
    
    setInitValV1(SP);
    
    
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
        [~,~,exitflag] = fmincon('costFunc_u_Straight',x,[],[],[],[],xhmin,xhmax,'geth',options); %,options
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
    Tendc(:,:,i)=Ttt_trocar*Tc;

end

%%
Terr_c=(Tend_m-Tendc);
dis_err_c=zeros(block_size,1);
for i =1 :block_size
    dis_err_c(i)=norm(Terr_c(1:3,4,i));

end
if(min(exitflag_kw)>0&&max(exitflag_kw)<2)
    f=mean(dis_err_c)*1000;
else
    f=rand(1)*1000;
    fprintf("\n\nnon-convergence.\n\n");
end

end