function [T_0, T_1, T_2]=getPose(name)
%--------- On trocar, marker's pose--------------------%
% to get  marker's pose
% ----Info
% By Keith W.
% Date 20201207
% Ver c1.0
% input: 不同组采集到的数据文件关键字
% output: 不同组的位姿数据
%
% level 当level=1时，此时为最低级，指的是以近端marker坐标系为目标坐标系
%       当level=2时，此时为中级，指的是以两marker连线为Z轴，当部分构型有近端marker坐标时采用。
%       当level=3时，此时为高级，指的是以亮marker连线为Z轴。
%-------------------------------------------------------------------------%
level = 3;

persistent T0;
persistent T1;
persistent T2;
if(nargin == 0)
    T_0=T0;
    T_1=T1;
    T_2=T2;
    return;
end


path_marker_1=['../test1206/',name,'/',name,'_1.log'];
path_marker_2=['../test1206/',name,'/',name,'_2.log'];
path_trocar_1=['../test1206/',name,'/coord_',name,'_1.log'];
path_trocar_2=['../test1206/',name,'/coord_',name,'_2.log'];

[Tcamera_trocar_1,z_dev_1,ang_dev_1]=getTrocar1Camera(load(path_trocar_1));
[Tcamera_trocar_2,z_dev_2,ang_dev_2]=getTrocar1Camera(load(path_trocar_2));
fprintf("->_1 两轴夹角为：%f°\n",ang_dev_1);
fprintf("->_1 两轴间距为：%fmm\n",z_dev_1);
fprintf("->_2 两轴夹角为：%f°\n",ang_dev_2);
fprintf("->_2 两轴间距为：%fmm\n",z_dev_2);


[Tcamera_marker_1, Tcamera_marker_p_1, Tcamera_marker_d_1, angle_block_1]=getMarker1Camera(load(path_marker_1));
[Tcamera_marker_2, Tcamera_marker_p_2, Tcamera_marker_d_2, angle_block_2]=getMarker1Camera(load(path_marker_2));
no_zero_ang_1=angle_block_1;
no_zero_ang_1(all(angle_block_1==0,2),:)=[];
no_zero_ang_2=angle_block_2;
no_zero_ang_2(all(angle_block_2==0,2),:)=[];
fprintf("-> 近端marker自身坐标系Z轴与连线轴夹角的最大值为：%f°\n",max(abs(no_zero_ang_1-mean(no_zero_ang_1))));
fprintf("-> 远端marker自身坐标系Z轴与连线轴夹角的最大值为：：%f°\n",max(abs(no_zero_ang_2-mean(no_zero_ang_2))));
block_size=max(size(Tcamera_marker_1));
Ttrocar_marker_1=zeros(4,4,block_size);
Ttrocar_marker_p_1=zeros(4,4,block_size);
Ttrocar_marker_d_1=zeros(4,4,block_size);
Ttrocar_marker_2=zeros(4,4,block_size);
Ttrocar_marker_p_2=zeros(4,4,block_size);
Ttrocar_marker_d_2=zeros(4,4,block_size);

T_verify=[    0.9998         0    0.0188         0
    0.0003    0.9998   -0.0183         0
   -0.0188    0.0183    0.9997         0
         0         0         0    1.0000];

xh=[0.0575   -0.3201   -0.4780    0.0014   -0.0093   -0.0040];
p=xh(1:3)/1000;
zyx=xh(4:6);
R=eul2rotm(zyx);
Ttt_trocar=[R,[0 0 0]';[0 0 0 1]];

% T_verify=eye(4);
Tcamera_trocar_1=Tcamera_trocar_1*T_verify*T_verify*inv(Ttt_trocar);
Tcamera_trocar_2=Tcamera_trocar_2*T_verify*T_verify*inv(Ttt_trocar);
for i = 1 :block_size
    if(~(Tcamera_marker_1(3,4,i)==0))
        Ttrocar_marker_1(:,:,i)=Tcamera_trocar_1\Tcamera_marker_1(:,:,i);
    end
    if(~(Tcamera_marker_p_1(3,4,i)==0))
        Ttrocar_marker_p_1(:,:,i)=Tcamera_trocar_1\Tcamera_marker_p_1(:,:,i);
    end
    if(~(Tcamera_marker_d_1(3,4,i)==0))
        Ttrocar_marker_d_1(:,:,i)=Tcamera_trocar_1\Tcamera_marker_d_1(:,:,i);
    end
    if(~(Tcamera_marker_2(3,4,i)==0))
        Ttrocar_marker_2(:,:,i)=Tcamera_trocar_2\Tcamera_marker_2(:,:,i);
    end
    if(~(Tcamera_marker_p_2(3,4,i)==0))
        Ttrocar_marker_p_2(:,:,i)=Tcamera_trocar_2\Tcamera_marker_p_2(:,:,i);
    end
    if(~(Tcamera_marker_d_2(3,4,i)==0))
        Ttrocar_marker_d_2(:,:,i)=Tcamera_trocar_2\Tcamera_marker_d_2(:,:,i);
    end

end


Ttrocar_marker=zeros(4,4,block_size);
Ttrocar_marker_p=zeros(4,4,block_size);
Ttrocar_marker_d=zeros(4,4,block_size);
for i =1:block_size
    if(Ttrocar_marker_1(3,4,i)==0)
        Ttrocar_marker(:,:,i)=Ttrocar_marker_2(:,:,i);
    else
        Ttrocar_marker(:,:,i)=Ttrocar_marker_1(:,:,i);
    end
    if(Ttrocar_marker_p_1(3,4,i)==0)
        Ttrocar_marker_p(:,:,i)=Ttrocar_marker_p_2(:,:,i);
    else
        Ttrocar_marker_p(:,:,i)=Ttrocar_marker_p_1(:,:,i);
    end
    if(Ttrocar_marker_d_1(3,4,i)==0)
        Ttrocar_marker_d(:,:,i)=Ttrocar_marker_d_2(:,:,i);
    else
        Ttrocar_marker_d(:,:,i)=Ttrocar_marker_d_1(:,:,i);
    end
end
T_0=Ttrocar_marker;
T_1=Ttrocar_marker_p;
T_2=Ttrocar_marker_d;
T0=T_0;T1=T_1;T2=T_2;
end