%% 
t=(80.5:10:180.5)*1e-3;
block_size=size(t,2);
[X,Y] = meshgrid(t);
Y=(Y-t(1))/(t(end)-t(1))*pi/2;
Y(1,:)=Y(1,:)+0.01;
exitflag_kw=zeros(block_size);
dis_err31_block=zeros(block_size);
dis_err21_block=zeros(block_size);
d_block=zeros(block_size);
% block_size=1;
figure;
%%
for i =1:block_size
    for j = 5:block_size
        LL=X(j,i);
        thetatheta=Y(j,i);
        SP.l=LL; % l>30
        
        
        SP.phi=0;
        SP.theta1=thetatheta;
        SP.delta1=0;
        SP.theta2=pi/2-thetatheta;
        SP.delta2=0;
        
        SP=setInitVal(SP);
        
        
        theta1=SP.theta1;
        xhmax=[10 40 10 10];
        xhmin=[-1e-9 -1e-9 -1e-9 0];
        x=(xhmin+xhmax)/2;
        options = optimoptions('fmincon','Algorithm','interior-point',"EnableFeasibilityMode",true); % 'Display','iter',,'PlotFcn','optimplotfirstorderopt','PlotFcn','optimplotfval'
        options.StepTolerance=1e-25;
        count=0;
        exitflag=0;
        while(exitflag~=1)
            
            [xh,yh,exitflag] = fmincon('costFunc_u_V2',x,[],[],[],[],xhmin,xhmax,'geth',options); %,options
            exitflag_kw(i,j)=exitflag;
            if(count>10)
                break;
            end
            count=count+1;
            x=rand(1,size(xhmin,2)).*(xhmax-xhmin)+xhmin;
        end
        xh=xh./[100 100 300 200];
        fprintf("i=%d,j=%d\n",i,j);
        if(xh(1)<1e-6)
            SP.d=xh(3);
        else
            SP.d=xh(4);
        end
        if(SP.Ls>1e-5)
            SP.thetasi=xh(1);
            SP.thetaso=xh(2);
            SP.theta1i=0;
            SP.theta1o=SP.theta1-SP.thetasi-SP.thetaso-SP.theta1i;
            SP.Lsi=SP.d/sin(SP.thetasi)*SP.thetasi;
            SP.L1i=0;
            SP.Lso=SP.Ls-SP.Lsi+SP.d;
            SP.L1o=SP.L1;
        else
            SP.thetaso=0;
            SP.Lso=0;
            SP.theta1i=xh(2);
            if(xh(4)-xh(3)<1e-5)
                SP.thetasi=0;
                SP.Lsi=0;  
                SP.theta1o=SP.theta1-SP.thetasi-SP.thetaso-SP.theta1i;
                SP.L1i=SP.d/sin(SP.theta1i)*SP.theta1i;
                SP.L1o=SP.l-SP.Lr-SP.L2-SP.L1i+SP.d;
                
            else
                SP.thetasi=xh(1);
                dsi=SP.d-xh(3);
                SP.Lsi=dsi/sin(SP.thetasi)*SP.thetasi;
                SP.L1i=xh(3)/cos(SP.thetasi)/sin(SP.theta1i)*SP.theta1i;
                SP.theta1o=SP.theta1-SP.thetasi-SP.thetaso-SP.theta1i;
                SP.L1o=SP.l-SP.Lr-SP.L2-SP.L1i+xh(3)/cos(SP.thetasi);
                
            end
        end
        
        
        
        setInitVal(SP);
        Tc=plotResult(0,"clearance=0");
        d_block(j,i)=SP.d;

        
        %%
        Tn0=plotResult_noClearance(0);
        Tn=plotResult_noClearance(0,0);
        dis_err21_block(j,i)=norm(Tc(1:3,4)-Tn(1:3,4));
        dis_err31_block(j,i)=norm(Tc(1:3,4)-Tn0(1:3,4));
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
