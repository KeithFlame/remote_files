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
[x,y,z]=stlread_keith('./end_effector/nh_1.STL');
% patch(x,y,z);
backcolor = 'white';
interval=1;
block_size=max(size(x));
t=1:block_size;
ti=find(mod(t,interval)==0);


patch(x(:,ti),y(:,ti),z(:,ti),'w','FaceAlpha',.5,'EdgeColor','black');

[x,y,z]=stlread_keith('./end_effector/nh_0.STL');
% patch(x,y,z);
% backcolor = 'white';
% interval=10;
block_size=max(size(x));
t=1:block_size;
ti=find(mod(t,interval)==0);
x=R*x;y=R*y;z=R*z;
patch(x(:,ti),y(:,ti),z(:,ti),'w','FaceAlpha',.5,'EdgeColor','black');
