% [x,y,z,c] = stlread('./end_effector/nh_0.STL');
%         		backcolor = 'white';
%         		patch(x,y,z,'w','FaceAlpha',.5,'EdgeColor','black');
%         		set(gca,'xtick',0:10:200);  
%        		 	set(gca,'ytick',0:10:90);
%        		 	set(gca,'ztick',-20:10:140);
%         		view(3);
% s
% cad2matdemo('./end_effector/nh_0.STL');


% TR = stlread_keith('./end_effector/nh_1.STL');
[x1,y1,z1]=stlread_keith('./end_effector/nh_1.STL');
[x2,y2,z2]=stlread_keith('./end_effector/nh_0.STL');
% patch(x,y,z);
backcolor = 'white';
interval=1;
block_size1=max(size(x1));
block_size2=max(size(x2));
R1=eul2rotm([ 0, 0 ,-pi/2]);
R2=eul2rotm([ 0, 0 ,pi]);
for j = 1:3
    for i = 1:block_size1
        p1=[x1(j,i); y1(j,i);z1(j,i)];
        p1=R1*p1;
        x1(j,i)=p1(1);y1(j,i)=p1(2);z1(j,i)=p1(3);
    end
    for i = 1:block_size2
        p2=[x2(j,i); y2(j,i);z2(j,i)];
        p2=R2*p2;
        x2(j,i)=p2(1);y2(j,i)=p2(2);z2(j,i)=p2(3);
    end
end
x1=x1-1.8;
% % % % z1=z1+17.98;y1=y1-2.4;
z1=z1+8.98;y1=y1-1.23;
z2=z2+17.98;y2=y2+3;x2=x2-2.63685;
figure;axis equal;grid on;view([90 0])
patch(x1,y1,z1,'w','FaceAlpha',.5,'EdgeColor','none','FaceColor',[0.5 0.5 0.5]);
patch(x2,y2,z2,'w','FaceAlpha',.5,'EdgeColor','none','FaceColor',[0.5 0.5 0.5]);

xlabel("x");
ylabel("y");
zlabel("z");