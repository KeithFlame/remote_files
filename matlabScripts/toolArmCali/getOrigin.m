function o=getOrigin(data_origin,z,P0)

direction=z;
P0=P0;
%% Origin
Porigin=data_origin(:,1:3);
dis_ori=zeros(size(Porigin,1),1);
for i = 1:size(Porigin,1)
    dis_ori(i)=norm(cross(direction,Porigin(i,:)-P0))/norm(direction);
end
[val,po]=min(dis_ori);
tt=Porigin(po,:);
Pori=P0'+(tt-P0)*direction.*direction;
o=Pori;%+[0.8 0 1.3]';
% o=[];
end