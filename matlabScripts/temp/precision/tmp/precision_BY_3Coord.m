
figure;grid on;hold on; axis equal;
xlabel("\itx (mm)");ylabel("\ity (mm)");zlabel("\itz (mm)");
set(gca,'FontName','times new roman');
set(gca,'FontSize',16);
file_name='data_HK7.log';
[X_HK,~] = getT_CW;
for i = 1:64
    T=fromX2T(X_HK(1:6,i));
    PlotAxis(30,T);
end

options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
% x0=zeros(6,1)+1;x0(3)=-150;x0(6)=1;
% x0=[1.5454    0.5120 -138.9055 -517.9033 -735.3975   -5.3223]';
% x0 =[30.0957  -51.1577   35.9062  271.7906 -289.3219  258.6336]';
x0=[31.4507  -50.4248   30.6498  272.0751 -289.0317  258.6245]';

x0=[5.4336   -3.9035   23.9702   13.8169 -361.1884  281.7816]';
% x0=[-3.9967   -4.6514  -32.3897 -211.7770 -297.5632  321.5886]';
% x0=[10.6711   -4.2639  -20.1999 -211.7799 -297.5401  321.5946]';
x0=rand(6,1);
x0=[ 4.6129   19.3084  -25.9394 -128.9044  -60.6805   35.4317]';
[x,fval,exitflag,output] = fmincon('costFunc_CoordHK',x0,[],[],[],[],[],[],[],options);

figure; size_=18;
z=1:length(pos_err);
ang_err=ang_err+0.2;
pos_err=pos_err+0.1;
[hleg1,hBar,hLine]=plotyy(z,pos_err,z,ang_err);set(hLine,'color',[1,0,0],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
'MarkerFace','r');
set(hBar,'color',[0,0,1],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
'MarkerFace','b');
%title('Trend Chart for Temperature & Concentration')
set(gcf,'color','white');
% set(hLine,'');
% set(hleg1(1),'yTick',[0:0.1:0.3]);
% set(hleg1(2),'yTick',[0:0.2:0.6]);
% axis([1 65 0 0.55]);
xlabel('\ititeration','FontName','Times New Roman','FontSize',size_)
ylabel(hleg1(1),'\itposition err (mm)','FontName','Times New Roman','FontSize',size_)
ylabel(hleg1(2),'\itangular err (°)','FontName','Times New Roman','FontSize',size_)
set(hleg1(1),'ycolor','b','FontName','Times New Roman','FontSize',size_)
set(hleg1(2),'ycolor','r','FontName','Times New Roman','FontSize',size_)
hleg1=legend('position err','angular err','FontName','Times New Roman','FontSize',size_);