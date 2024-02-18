file_name='data_HK.log';
X = data_processing_HK(file_name);

z = 1:50;
p1mean=mean(X(1:3,1:50)');
p2mean=mean(X(1:3,51:100)');
dis1=zeros(50,3);dis2=dis1;
ang = zeros(100,1);
for i = 1:size(X,2)
    ang(i) = norm(X(4:6,i));
    if(i<51)
        dis1(i,:)=X(1:3,i)'-p1mean;
    end
    if(i>50)
        dis2(i-50,:)=X(1:3,i)'-p2mean;
    end
end
ang1 = ang(1:50)-mean(ang(1:50));
ang2 = ang(51:100)-mean(ang(51:100));

figure;size_ = 26; 
[hleg1,hBar,hLine]=plotyy(z,dis1(:,1),z,dis1(:,2));
set(hLine,'color',[1,0,0],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
    'MarkerFace','r');
set(hBar,'color',[0,0,1],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
    'MarkerFace','b');
%title('Trend Chart for Temperature & Concentration')
set(gcf,'color','white')
xlabel('\ititeration','FontName','Times New Roman','FontSize',size_)
ylabel(hleg1(1),'\itX-p noise (mm)','FontName','Times New Roman','FontSize',size_)
ylabel(hleg1(2),'\itY_p noise (mm)','FontName','Times New Roman','FontSize',size_)
set(hleg1(1),'ycolor','b','FontName','Times New Roman','FontSize',size_)
set(hleg1(2),'ycolor','r','FontName','Times New Roman','FontSize',size_)
hleg1=legend('X-p noise','Y-p noise','FontName','Times New Roman','FontSize',size_);


figure;size_ = 26; 
[hleg1,hBar,hLine]=plotyy(z,dis1(:,3),z,ang1);
set(hLine,'color',[1,0,0],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
    'MarkerFace','r');
set(hBar,'color',[0,0,1],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
    'MarkerFace','b');
%title('Trend Chart for Temperature & Concentration')
set(gcf,'color','white')
xlabel('\ititeration','FontName','Times New Roman','FontSize',size_)
ylabel(hleg1(1),'\itZ_p noise (mm)','FontName','Times New Roman','FontSize',size_)
ylabel(hleg1(2),'\itangular noise (°)','FontName','Times New Roman','FontSize',size_)
set(hleg1(1),'ycolor','b','FontName','Times New Roman','FontSize',size_)
set(hleg1(2),'ycolor','r','FontName','Times New Roman','FontSize',size_)
hleg1=legend('Z_p noise','angular noise','FontName','Times New Roman','FontSize',size_);



figure;size_ = 26; 
[hleg1,hBar,hLine]=plotyy(z,dis2(:,1),z,dis2(:,2));
set(hLine,'color',[1,0,0],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
    'MarkerFace','r');
set(hBar,'color',[0,0,1],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
    'MarkerFace','b');
%title('Trend Chart for Temperature & Concentration')
set(gcf,'color','white')
xlabel('\ititeration','FontName','Times New Roman','FontSize',size_)
ylabel(hleg1(1),'\itX-p noise (mm)','FontName','Times New Roman','FontSize',size_)
ylabel(hleg1(2),'\itY_p noise (mm)','FontName','Times New Roman','FontSize',size_)
set(hleg1(1),'ycolor','b','FontName','Times New Roman','FontSize',size_)
set(hleg1(2),'ycolor','r','FontName','Times New Roman','FontSize',size_)
hleg1=legend('X-p noise','Y-p noise','FontName','Times New Roman','FontSize',size_);

figure;size_ = 26; 
[hleg1,hBar,hLine]=plotyy(z,dis2(:,3),z,ang2);
set(hLine,'color',[1,0,0],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
    'MarkerFace','r');
set(hBar,'color',[0,0,1],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
    'MarkerFace','b');
%title('Trend Chart for Temperature & Concentration')
set(gcf,'color','white')
xlabel('\ititeration','FontName','Times New Roman','FontSize',size_)
ylabel(hleg1(1),'\itZ_p noise (mm)','FontName','Times New Roman','FontSize',size_)
ylabel(hleg1(2),'\itangular noise (°)','FontName','Times New Roman','FontSize',size_)
set(hleg1(1),'ycolor','b','FontName','Times New Roman','FontSize',size_)
set(hleg1(2),'ycolor','r','FontName','Times New Roman','FontSize',size_)
hleg1=legend('Z_p noise','angular noise','FontName','Times New Roman','FontSize',size_);