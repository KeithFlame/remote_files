% x=90:0.1:98;
z=111:0.1:120;
d = 34*(z+50)./(2*z-86.5-80-40);
theta1 = 2*atand((z+50)./d);

y=-30:(7/90):-23;
d1 = (y-54)./(2*y+41)*33+34;
theta2 = 2*atand((54-y)./(d1-34));


figure;size_ = 26; 
[hleg1,hBar,hLine]=plotyy(z,d,z,theta1);
set(hLine,'color',[1,0,0],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
    'MarkerFace','r');
set(hBar,'color',[0,0,1],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
    'MarkerFace','b');
%title('Trend Chart for Temperature & Concentration')
set(gcf,'color','white')
xlabel('\itz (mm)','FontName','Times New Roman','FontSize',size_)
ylabel(hleg1(1),'\itd (mm)','FontName','Times New Roman','FontSize',size_)
ylabel(hleg1(2),'\itHfov (°)','FontName','Times New Roman','FontSize',size_)
set(hleg1(1),'ycolor','b','FontName','Times New Roman','FontSize',size_)
set(hleg1(2),'ycolor','r','FontName','Times New Roman','FontSize',size_)
hleg1=legend('d','Hfov','FontName','Times New Roman','FontSize',size_);
figure;
[hleg1,hBar,hLine]=plotyy(y,d1,y,theta2);
set(hLine,'color',[1,0,0],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
    'MarkerFace','r');
set(hBar,'color',[0,0,1],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
    'MarkerFace','b');
%title('Trend Chart for Temperature & Concentration')
set(gcf,'color','white')
xlabel('\itz (mm)','FontName','Times New Roman','FontSize',size_)
ylabel(hleg1(1),'\itd (mm)','FontName','Times New Roman','FontSize',size_)
ylabel(hleg1(2),'\itVfov (°)','FontName','Times New Roman','FontSize',size_)
set(hleg1(1),'ycolor','b','FontName','Times New Roman','FontSize',size_)
set(hleg1(2),'ycolor','r','FontName','Times New Roman','FontSize',size_)
hleg1=legend('d','Vfov','FontName','Times New Roman','FontSize',size_);

hx = 10*(d+67).*tand(theta1/2);
hy = 10*(d1+67).*tand(theta2/2);
figure;
[hleg1,hBar,hLine]=plotyy(d,hx,d,hy);
set(hLine,'color',[1,0,0],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
    'MarkerFace','r');
set(hBar,'color',[0,0,1],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
    'MarkerFace','b');
%title('Trend Chart for Temperature & Concentration')
set(gcf,'color','white')
xlabel('\itd (mm)','FontName','Times New Roman','FontSize',size_)
ylabel(hleg1(1),'\itpixel','FontName','Times New Roman','FontSize',size_)
ylabel(hleg1(2),'\itpixel','FontName','Times New Roman','FontSize',size_)
set(hleg1(1),'ycolor','b','FontName','Times New Roman','FontSize',size_)
set(hleg1(2),'ycolor','r','FontName','Times New Roman','FontSize',size_)
hleg1=legend('\itpixel-w','\itpixel-h','FontName','Times New Roman','FontSize',size_);