clear,clc;
%%
SP.psi.l=0.1227; % l>30
SP.psi.phi=0;
SP.psi.theta1=pi/2;
SP.psi.delta1=0;
SP.psi.theta2=0;
SP.psi.delta2=0;

SP=setInitValV1(SP);


theta1=SP.psi.theta1;
xhmax=[10 40 10 10];
xhmin=[-1e-3 -1e-3 -1e-3 0];
options = optimoptions('fmincon','Algorithm','interior-point','display','iter',"EnableFeasibilityMode",true); % 'Display','iter',,'PlotFcn','optimplotfirstorderopt','PlotFcn','optimplotfval'
options.StepTolerance=1e-25;
options.OptimalityTolerance=5e-6;
x=(xhmin+xhmax)/2;
x=rand(1,size(xhmin,2)).*(xhmax-xhmin)+xhmin;
% x=[0    0    0.0049    0.0449].*[100 100 300 200];
[xh,yh,exitflag] = fmincon('costFunc_u_Straight',x,[],[],[],[],xhmin,xhmax,'geth',options); %,options
xh=xh./[100 100 300 200];
%     count=1;
%     while(exitflag~=1)
%         [xh,yh,exitflag] = fmincon('costFunc_u_Straight',x,[],[],[],[],xhmin,xhmax,'geth',options); %,options
%         if(count>1000)
%             fprintf('\n\ntem_psi: %f,%f,%f,%f,%f,%f\n\nexitflag:%d',tem_psi,exitflag);
%             break;
%         end
%         count=count+1;
%         x=rand(1,size(xhmin,2)).*(xhmax-xhmin)+xhmin;
%     end
figure;
Tc=plotResult(1,"");
% Tn_d=plotResult_noClearance(1);
Tn=plotResult_noClearance(0,0);
SP=getSPV1;
axis([-5 +5 -5 +5 -25 +5])
% axis([Tc(1,4)-5e-6 Tc(1,4)+5e-6 Tc(2,4)-5e-6 Tc(2,4)+5e-6 Tc(3,4)-5e-6 Tc(3,4)+5e-6])

tt=[SP.trocar.d exitflag ]
Tn(1:3,4)'-Tc(1:3,4)'
leg.ItemTokenSize = [3,1];
F=getframe(gcf);
set(gca,'FontSize',8);
set(gcf,'Position',[500,500,400,200], 'color','w');
imwrite(F.cdata,'F:\synchronism_plus\synchronism_plus\好好写文章\好好写文章01\pic\final61.tiff');