% 坐标系验证
[Tt4, Tt14, Tt40] = registration_CW2;
% [Tt71, Tt191, Tt311] = registration_CW('Measure7',2);
Tt4=eye(4);
Tt14=[-0.00362543009794170	-0.999988701839639	0.00307448364421231	18.4181734989840
0.00575201421647198	0.00305359942347655	0.999978794706675	28.1410941662860
-0.999976885027378	0.00364303769326280	0.00574087861803602	-9.37588713914540
0	0	0	1];
Tt40=[-0.00729589852429502	0.00544085683414103	-0.999958582613117	-10.7318086877484
-0.999821937958613	-0.0174428342946687	0.00719999364245327	19.3690482615277
-0.0174029377234370	0.999833058369586	0.00556714917305623	-27.9159096573097
0	0	0	1];
err=zeros(64,2);

p4 = zeros(64,7);m4=1;
p14 = zeros(64,7);m14=1;
p40 = zeros(64,7);m40=1;
T4=zeros(6,64);
T_14=zeros(6,64);
T14=T4;
T40=T4;
for i = 1:64
    str = ['pt',num2str(i-1)];
    t4 = eye(4);
    t14 = eye(4);
    t_14 = eye(14);
    t40 = eye(4);
    [C4, C14, C40]=getData71931(str,1);
    if(C4(1,3)<-10)
        p4(m4,:)=getDistance(C4);
        m4=m4+1;
        t4=getCoord(C4)*Tt4;
    end
    if(C14(1,3)<-10)
        p14(m14,:)=getDistance(C14);
        m14=m14+1;
        t_14=getCoord(C14);
        t14=getCoord(C14)/Tt14;
    end
    if(C40(1,3)<-10)
        p40(m40,:)=getDistance(C40);
        m40=m40+1;
        t40=getCoord(C40)/Tt40;
    end
%     if(C4(1,3)<-10&&C40(1,3)<-10)
%         T4=getCoord(C4);
%         T40=getCoord(C40);
%         t4=T4;
%         T01 = T4/Tt4;
%         T02 = T40/Tt40;
%     elseif(C14(1,3)<-10&&C40(1,3)<-10)
%         T14=getCoord(C14);
%         T40=getCoord(C40);
%         T01 = T14/Tt14;
%         T02 = T40/Tt40;
%     elseif(C4(1,3)<-10&&C14(1,3)<-10)
%         T4=getCoord(C4);
%         T14=getCoord(C14);
%         T01 = T4/Tt4;
%         T02 = T14/Tt14;
%     else
%         T01=eye(4);
%         T02=eye(4);
%     end

%     err(i+1,1)=norm(T01(1:3,4)-T02(1:3,4));
%     axang=rotm2axang(T02(1:3,1:3)'*T01(1:3,1:3));
%     err(i+1,2)=axang(4)*180/pi;
    T4(:,i)=fromT2X(t4);
    T14(:,i)=fromT2X(t14);
    T40(:,i)=fromT2X(t40);
    T_14(:,i)=fromT2X(t_14);
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
figure(11); hold on;
plot(max_err4,'color',[1,0,0],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
    'MarkerFace','r');
plot(max_err14,'color',[0,1,0],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
    'MarkerFace','g');
plot(max_err40,'color',[0,0,1],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
    'MarkerFace','b');
xlabel("编号")
ylabel("mm")
xlabel("编号")
ylabel("mm")
set(gca,'FontSize',14);
% mean(err);
% z = 1:size(err,1);
% figure;size_ = 26; 
% [hleg1,hBar,hLine]=plotyy(z,err(:,1),z,err(:,2));
% set(hLine,'color',[1,0,0],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
%     'MarkerFace','r');
% set(hBar,'color',[0,0,1],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
%     'MarkerFace','b');
% %title('Trend Chart for Temperature & Concentration')
% set(gcf,'color','white');
% % xlim([1 12]);
% xlabel('\ititeration','FontName','Times New Roman','FontSize',size_)
% ylabel(hleg1(1),'\itposition noise (mm)','FontName','Times New Roman','FontSize',size_)
% ylabel(hleg1(2),'\itangular noise (°)','FontName','Times New Roman','FontSize',size_)
% set(hleg1(1),'ycolor','b','FontName','Times New Roman','FontSize',size_)
% set(hleg1(2),'ycolor','r','FontName','Times New Roman','FontSize',size_)
% hleg1=legend('position noise','angular noise','FontName','Times New Roman','FontSize',size_);


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