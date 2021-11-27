

T_left = getT('left');
T_right = getT('right');
T_relative=zeros(4,4,size(T_left,3));
rot_z_pi=[-1 0 0 0;0 -1 0 0;0 0 1 0;0 0 0 1];
for i = 1 :size(T_left,3)
    if(i>15)
        
        T_relative(:,:,i)=T_left(:,:,i)\T_right(:,:,i);
        if(T_relative(1,4,i)>0)
            T_relative(:,:,i)=T_relative(:,:,i)*rot_z_pi;
        end
    else
        T_relative(:,:,i)=T_right(:,:,i)\T_left(:,:,i);
        if(T_relative(1,4,i)<0)
            T_relative(:,:,i)=T_relative(:,:,i)*rot_z_pi;
        end
    end
end

T1=reshape(T_relative(1:3,4,1:size(T_left,3)/2),[3 15])';
T2=reshape(T_relative(1:3,4,size(T_left,3)/2+1:size(T_left,3)),[3 15])';
for i = 1:size(T1)
    if(T1(i,3)<50||T1(i,3)>60)
        T1(i,:)=[0 0 0];
    end
    if(T2(i,3)<155||T2(i,3)>165)
        T2(i,:)=[0 0 0];
    end
end
T1(T1(:,2)==0,:)=[];
T2(T2(:,2)==0,:)=[];