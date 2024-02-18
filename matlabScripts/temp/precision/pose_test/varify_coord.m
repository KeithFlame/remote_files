% 坐标系验证
[Tt4, Tt14, Tt40] = registration_CW('../0826/M1',1);
% [Tt71, Tt191, Tt311] = registration_CW('Measure7',2);

err=zeros(81,2);
[C4, ~, C40]=getData71931('../0826/M4',1);
T4=getCoord(C4);
T40=getCoord(C40);
T01 = T4/Tt4;
T02 = T40/Tt40;
err(1,1)=norm(T01(1:3,4)-T02(1:3,4));
axang=rotm2axang(T02(1:3,1:3)'*T01(1:3,1:3));
err(1,2)=axang(4)*180/pi;
% str={"M11","M12","M13","M14","M15"};
p4 = zeros(70,7);m4=1;
p14 = zeros(70,7);m14=1;
p40 = zeros(70,7);m40=1;
for i = 1:70
    str = ['../0826/P',num2str(i)];
    [C4, C14, C40]=getData71931(str,1);
    if(C4(1,3)<-10)
        p4(m4,:)=getDistance(C4);
        m4=m4+1;
    end
    if(C14(1,3)<-10)
        p14(m14,:)=getDistance(C14);
        m14=m14+1;
    end
    if(C40(1,3)<-10)
        p40(m40,:)=getDistance(C40);
        m40=m40+1;
    end
    if(C4(1,3)<-10&&C40(1,3)<-10)
        T4=getCoord(C4);
        T40=getCoord(C40);
        T01 = T4/Tt4;
        T02 = T40/Tt40;
    elseif(C14(1,3)<-10&&C40(1,3)<-10)
        T14=getCoord(C14);
        T40=getCoord(C40);
        T01 = T14/Tt14;
        T02 = T40/Tt40;
    elseif(C4(1,3)<-10&&C14(1,3)<-10)
        T4=getCoord(C4);
        T14=getCoord(C14);
        T01 = T4/Tt4;
        T02 = T14/Tt14;
    else
        T01=eye(4);
        T02=eye(4);
    end

    err(i+1,1)=norm(T01(1:3,4)-T02(1:3,4));
    axang=rotm2axang(T02(1:3,1:3)'*T01(1:3,1:3));
    err(i+1,2)=axang(4)*180/pi;
end
err(all(err==0,2),:) = [];
p4(all(p4==0,2),:) = [];
p14(all(p14==0,2),:) = [];
p40(all(p40==0,2),:) = [];

max_err4=zeros(7,1);
max_err14=zeros(7,1);
max_err40=zeros(7,1);
for i =1:7
    max_err4(i)=max(p4(:,i))-min(p4(:,i));
    max_err14(i)=max(p14(:,i))-min(p14(:,i));
    max_err40(i)=max(p40(:,i))-min(p40(:,i));
end

figure(10); hold on;
plot(max_err4,'color',[1,0,0],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
    'MarkerFace','r');
plot(max_err14,'color',[0,1,0],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
    'MarkerFace','g');
plot(max_err40,'color',[0,0,1],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
    'MarkerFace','b');
xlabel("编号")
ylabel("mm")
set(gca,'FontSize',14);
% mean(err);
z = 1:19;
figure;size_ = 26; 
[hleg1,hBar,hLine]=plotyy(z,err(:,1),z,err(:,2));
set(hLine,'color',[1,0,0],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
    'MarkerFace','r');
set(hBar,'color',[0,0,1],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
    'MarkerFace','b');
%title('Trend Chart for Temperature & Concentration')
set(gcf,'color','white');
% xlim([1 12]);
xlabel('\ititeration','FontName','Times New Roman','FontSize',size_)
ylabel(hleg1(1),'\itposition noise (mm)','FontName','Times New Roman','FontSize',size_)
ylabel(hleg1(2),'\itangular noise (°)','FontName','Times New Roman','FontSize',size_)
set(hleg1(1),'ycolor','b','FontName','Times New Roman','FontSize',size_)
set(hleg1(2),'ycolor','r','FontName','Times New Roman','FontSize',size_)
hleg1=legend('position noise','angular noise','FontName','Times New Roman','FontSize',size_);






function p=getDistance(C)
p=zeros(1,7);
p(1)=norm(C(1,:)-C(2,:));
p(2)=norm(C(1,:)-C(3,:));
p(3)=norm(C(1,:)-C(4,:));
p(4)=norm(C(1,:)-C(5,:));
p(5)=norm(C(1,:)-C(6,:));
p(6)=norm(C(1,:)-C(7,:));
p(7)=norm(C(1,:)-C(8,:));
end