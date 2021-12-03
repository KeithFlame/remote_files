%% 
t=(31.5:5:181.5)*1e-3;
block_size=size(t,2);
[X,Y] = meshgrid(t);
Y=(Y-t(1))/(t(end)-t(1))*pi/2;
Y(1,:)=Y(1,:)+0.05;
exitflag_kw=zeros(block_size)+5;
dis_err31_block=zeros(block_size);
dis_err21_block=zeros(block_size);
d_block=zeros(block_size);
% block_size=1;
figure;
%%
for i =1:block_size
    for j = 1:block_size
        LL=X(j,i);
        thetatheta=Y(j,i);
        SP.psi.l=LL; % l>30
        SP.psi.phi=0;
        SP.psi.theta1=thetatheta;
        SP.psi.delta1=0;
        SP.psi.theta2=pi/2-SP.psi.theta1;
        SP.psi.delta2=0;
        
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
            exitflag_kw(j,i)=exitflag;
            if(count>20)
                break;
            end
            count=count+1;
            x=rand(1,size(xhmin,2)).*(xhmax-xhmin)+xhmin;
        end
        SP=getSPV1;
        Tc=plotResult(0);
        Tn0=plotResult_noClearance(0);
        Tn=plotResult_noClearance(0,0);
        dis_err21_block(j,i)=norm(Tc(1:3,4)-Tn(1:3,4));
        dis_err31_block(j,i)=norm(Tc(1:3,4)-Tn0(1:3,4));
        d_block(j,i)=SP.trocar.d;
    end
end
%%
C=X.*Y;
figure;mesh(X,Y,dis_err21_block,C)
xlabel('l (m)');ylabel('\theta_1 (rad)');zlabel('dis error (m)');title('\itline2 vs line1');
figure;mesh(X,Y,dis_err31_block,C)
xlabel('l (m)');ylabel('\theta_1 (rad)');zlabel('dis error (m)');title('\itline3 vs line1');
figure;mesh(X,Y,d_block,C)
xlabel('l (m)');ylabel('\theta_1 (rad)');zlabel('d (m)');title('\itd')
figure;plot(X(1,:),d_block(end,:));
figure;plot(Y(1,:),d_block(:,1));
