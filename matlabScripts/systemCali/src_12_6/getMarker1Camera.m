function [Tcamera_marker, Tcamera_marker_p, Tcamera_marker_d, angle_block]=getMarker1Camera(M)
%--------- stereo-camera coordinate marker's pose--------------------%
% to get stereo-camera marker's pose
% ----Info
% By Keith W.
% Date 20201207
% Ver c1.0
% input: 不同组采集到的数据
% output: 不同组的位姿文件
%-------------------------------------------------------------------------%
block_size=max(size(M)/4);
Tcamera_marker=zeros(4,4,block_size);
Tcamera_marker_p=zeros(4,4,block_size);
Tcamera_marker_d=zeros(4,4,block_size);
angle_block=zeros(block_size,1);
for i =1:block_size
    P_tem_1=M(i*4-1,[1 2 3]);
    P_tem_2=M(i*4,[1 2 3]);
    if(P_tem_1(3)<50||P_tem_2(3)<50)
        Tcamera_marker(:,:,i)=eye(4);
        Tcamera_marker_p(:,:,i)=eye(4);
        Tcamera_marker_d(:,:,i)=eye(4);
        continue;
    end
    z=(P_tem_2-P_tem_1)/norm(P_tem_2-P_tem_1);
    z=reshape(z,[3 1]);
    x=[1 0 0]';
    y=cross(z,x)/norm(cross(z,x));
    x=cross(y,z)/norm(cross(y,z));
    R=[x y z];
    Tcamera_marker(1:3,1:3,i)=R;
    P=reshape(P_tem_1,[3 1]);
    Tcamera_marker(1:3,4,i)=P;
    Tcamera_marker(4,4,i)=1;
    
    %
    quat1=M(i*4-1,[4 5 6 7]);
    quat1=quat1/norm(quat1);
    Rtrocar_marker_proximal=quat2rotm(quat1);
    Tcamera_marker_p(:,:,i)=[Rtrocar_marker_proximal P_tem_1';0 0 0 1 ];
    quat2=M(i*4,[4 5 6 7]);
    quat2=quat2/norm(quat2);
    Rtrocar_marker_rural=quat2rotm(quat2);
    Tcamera_marker_d(:,:,i)=[Rtrocar_marker_rural P_tem_2';0 0 0 1 ];
    axang=rotm2axang(Rtrocar_marker_rural'*Rtrocar_marker_proximal);
    angle_block(i)=axang(4)*180/pi;
    %

end
end


