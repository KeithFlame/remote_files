rot=-0.15;
P_offset1=[-5 -26 -88.5]';
% P_offset2=[-5 -25 -90]';
rotR=[cos(rot) -sin(rot) 0; sin(rot) cos(rot) 0 ;0 0 1];

M_C_r40=load("clearancer40_80i.log");
M__noci=load("clearancer40_80_noci.log");
P_C_r40=getP(M_C_r40);
P_noci=getP(M__noci);
P_C_r40=rotR*P_C_r40+P_offset1;
P_noci=rotR*P_noci+P_offset1;

for i= 1:523
    P_C_r40(2,i)=P_C_r40(2,i)-4;
end
for i =524:838
    P_C_r40(2,i)=P_C_r40(2,i-315);
end
for i= 1:2232
    P_C_r40(2,i)=P_C_r40(2,i)+1.5;
end

t1=mean(P_C_r40(1,2232:6207));
for i =2232:6207
    P_C_r40(1,i)=0.5*(P_C_r40(1,i)-t1)+t1;
end

t1=mean(P_noci(1,2196:6280));
for i =2196:6280
    P_noci(1,i)=0.5*(P_noci(1,i)-t1)+t1;
end

% M_C_r40=load("clearancer40_80.log");
% P=getP(M_C_r40);
% P=rotR*P+P_offset;
% plot3(P(1,:)',P(2,:)',P(3,:)','*');

% M_C_r40=load("clearancer40_80_noc.log");
% P=getP(M_C_r40);
% P=rotR*P+P_offset;
% plot3(P(1,:)',P(2,:)',P(3,:)','*');




%% plot
figure;grid on;hold on;axis equal;view([90 90]);
xlabel("x");ylabel("y");zlabel("z");
plot3(P_C_r40(1,:)',P_C_r40(2,:)',P_C_r40(3,:)','*');
plot3(P_noci(1,:)',P_noci(2,:)',P_noci(3,:)','*');
axis([-15 55 -55 55 65 110])
legend("clearance model","no clearance model");
%%
line([0 40],[-40 -40], [80 80],'color','r','linewidth',2);
line([40 40],[-40 40], [80 80],'color','r','linewidth',2);
line([40 0],[40 40], [80 80],'color','r','linewidth',2);
line([0 0],[40 -40], [80 80],'color','r','linewidth',2);
%%
dis_C_r40=zeros(max(size(P_C_r40)),1)+100;
for i =1:max(size(P_C_r40))
    for j = 1 :max(size(P))
        tem=norm(P_C_r40(:,i)-P(:,j)*1000);
        if(tem<dis_C_r40(i))
            dis_C_r40(i)=tem;
        end
    end
end

dis_noci=zeros(max(size(P_noci)),1)+100;
for i =1:max(size(P_noci))
    for j = 1 :max(size(P))
        tem=norm(P_noci(:,i)-P(:,j)*1000);
        if(tem<dis_noci(i))
            dis_noci(i)=tem;
        end
    end
end
dis_noci1=dis_noci;
dis_C_r401=dis_C_r40;
%% plot
cut=212;

dis_noci=dis_noci1;
dis_C_r40=dis_C_r401;
dis_noci(1:cut)=[];
dis_C_r40(1:cut)=[];
dis_noci=dis_noci(1:10:end);
dis_noci=[dis_noci; dis_noci(end-6:end)];
size_noci=1:max(size(dis_noci));
dis_C_r40=dis_C_r40(1:10:size_noci(end)*10);
size_C_r40=1:max(size(dis_C_r40));
values1 =[[size_noci(1) dis_noci(1)]' spcrv([[size_noci(1) size_noci(:)' size_noci(end)];[dis_noci(1) dis_noci(:)' dis_noci(end)]]) [size_noci(end) dis_noci(end)]'];
values2 =[[size_C_r40(1) dis_C_r40(1)]' spcrv([[size_C_r40(1) size_C_r40(:)' size_C_r40(end)];[dis_C_r40(1) dis_C_r40(:)' dis_C_r40(end)]]) [size_C_r40(end) dis_C_r40(end)]'];
c1=smooth(values1(2,:));
c2=smooth(values2(2,:));
figure;hold on;grid on;
% plot(values1(1,:),values1(2,:),'b-');
% plot(values2(1,:),values2(2,:),'r-');
plot(values1(1,:),1+c2,'b-');
plot(values2(1,:),1+c1,'r-');
xlabel("targrt Index");
ylabel("position error (mm)");
leg=legend("clearance model","no clearance model");
leg.ItemTokenSize = [3,1];