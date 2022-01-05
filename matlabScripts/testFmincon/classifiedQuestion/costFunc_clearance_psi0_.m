function f=costFunc_clearance_psi0_(x)

name='59_2';
[Tend_m,Psi]=getEnd(name);
Psi=Psi';

x=x./[10 0.5 10 1 10 1]*pi;
% psi0=[0.0055    0.0005    0.0289    0.0020   -0.0000    0.0034]';
psi0=x;
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
% d_block=zeros(block_size,1);
Tendc=zeros(4,4,block_size);
% Tendn=zeros(4,4,block_size);
% Tendn_no=zeros(4,4,block_size);
% L1x=-1.9744*1e-3-3.7e-3;
L1x=-5.5*1e-3;
dttc=9.9999*1e-3;
tic;
for i = 1:block_size

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
    
%     SP=setInitValV1(SP);
    
    
    SP=setInitValV2(SP);
    OP=setOP;
    r_zero_pose=getZeroPose(SP.psi.theta1);
    flag_k=1;
    if(r_zero_pose>SP.trocar.c/2)
        ij=0;
        while(ij<20)
            if(SP.psi.l>SP.structure.L1+SP.structure.L2+SP.structure.Lr)
                [~,~,exitflag_1]=fmincon('costFunc_1',OP.x1,[],[],[],[],OP.xhmin1,OP.xhmax1,'nonlcon_1',OP.options);
                OP.x1=rand(1,max(size(OP.xhmax1))).*(OP.xhmin1-OP.xhmax1)+OP.xhmax1;
                flag_k=1;
            else

                [xh,~,exitflag_1]=fmincon('costFunc_2',OP.x2,[],[],[],[],OP.xhmin2,OP.xhmax2,'nonlcon_2',OP.options);
                OP.x2=rand(1,max(size(OP.xhmax2))).*(OP.xhmin2-OP.xhmax2)+OP.xhmax2;
                flag_k=2;
                if(xh(1)/50+xh(2)/10>SP.structure.L1)
                    [~,~,exitflag_1]=fmincon('costFunc_3',OP.x3,[],[],[],[],OP.xhmin3,OP.xhmax3,'nonlcon_3',OP.options);
                    OP.x3=rand(1,max(size(OP.xhmax3))).*(OP.xhmin3-OP.xhmax3)+OP.xhmax3;
                    flag_k=3;
                end
            end
            ij=ij+1;
            if(exitflag_1==1)
                break;
            end
        end
        exitflag_kw(i)=exitflag_1;
        Tc=plotResult(0,flag_k,"");
    else
        getSP_1(OP.x1);
        exitflag_kw(i)=1;
        Tc=plotResult_noClearance(0,flag_k,dttc*5);
    end
%     Tn=plotResult_noClearance(0,flag_k,dttc);
%     Tn_no=plotResult_noClearance(0,flag_k,0);
%     Tm=Tend_m(:,:,i);
    
    Tendc(:,:,i)=Ttt_trocar*Tc;
%     Tendn(:,:,i)=Ttt_trocar*Tn;
%     Tendn_no(:,:,i)=Ttt_trocar*Tn_no;
%     datestr(now,'mmmm dd,yyyy HH:MM:SS.FFF AM'),fprintf(":    %d \n",i);
end
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