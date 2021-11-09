str='/camera1023_cali';
path=load('../conf/configurationData/filePath.log');

is_write=1;

data_origin=load(['./data',str,'\O.log']);
data_Y=load(['./data',str,'\Y.log']);
data_Z=load(['./data',str,'\Z.log']);

[z, cter_z,P0]=getZAxis_vt(data_Z);
if(z(1)<0)
    z=-z;
end
% [o1,cter]=getOriginV2(data_origin);
o1=getOrigin(data_origin,z,P0);
o=-z*3.5+o1;

Tm1=load(['../conf/configurationData/arm',num2str(path),'/varifyTrocarCoords_C1.log']);
Tm2=load(['../conf/configurationData/arm',num2str(path),'/varifyTrocarCoords_C2.log']);

Tset=[Tm1 Tm2];
Tset=Tset(1:19,:);
interval=[1:6 7:19];
[p2, r2]=spatialCircleFitV2(Tset(interval,9:11));
[p1, r1]=spatialCircleFitV2(Tset(interval,1:3));
% [center,radius,normal] = spaceCircleFit(Tset(interval,9:11))
Z_old=z;
z=(p2-o)/norm(p2-o);
trocar_new_vs_old=acosd(dot(Z_old,z));
% z=Z_old;
[y,cter_y]=getYAxis(data_Y);
% o=getOrigin(data_origin,z,P0);

x=cross(y,z)/norm(cross(y,z));

y=cross(z,x)/norm(cross(z,x));

R=[x y z];

Porigin=o;
T=[R,Porigin;[0 0 0 1]];

Tst2tc=T
Twrite=reshape(Tst2tc,[1 16]);
if(is_write)
fileConfig = fopen('../conf/matlab/Tst2tc_new.log','w');
fprintf(fileConfig,'%6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f\n',Twrite);
fclose(fileConfig);

fpathResult=['../conf/configurationData/arm',num2str(path),'/Tst2tc.log'];
fileConfig = fopen(fpathResult,'w');
fprintf(fileConfig,'%6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f\n',Twrite);
fclose(fileConfig);
end
%%

gamma1_0=0;
tT=-180:20:180;
tT=tT(interval)-gamma1_0;
Pcamera_marker_1=Tset(interval,1:3);
Pcamera_marker_2=Tset(interval,9:11);
Pl_limit_marker_1=zeros(size(tT,2),3);
Pl_limit_marker_2=zeros(size(tT,2),3);
for i  =1 :size(tT,2)
    Ttrocar2rotation=[cosd(tT(i)) -sind(tT(i)) 0 0;sind(tT(i)) cosd(tT(i)) 0 0;0 0 1 0;0 0 0 1];
    Ttem=inv(Tst2tc*Ttrocar2rotation);
    P=(Ttem*[Pcamera_marker_1(i,:) 1]')';
    Pl_limit_marker_1(i,:)=P(1:3);
    P=(Ttem*[Pcamera_marker_2(i,:) 1]')';
    Pl_limit_marker_2(i,:)=P(1:3);
end
% [fc1, fr1]=spaceBallFit(Ptrocar_marker_1);
% [fc2, fr2]=spaceBallFit(Ptrocar_marker_2);


[fc1, fr1]=getOriginV2(Pl_limit_marker_1);
[fc2, fr2]=getOriginV2(Pl_limit_marker_2);
% plotCircle(fc1,fr1,Pl_limit_marker_1,'F1')
% plotCircle(fc2,fr2,Pl_limit_marker_2,'F2')

fc1=reshape(fc1,[3 1]);
fc2=reshape(fc2,[3 1]);
n_Lm_marker=(fc2-fc1)/norm((fc2-fc1));
while(norm(n_Lm_marker(1:2))>0.02)
    n_Lm_marker=(n_Lm_marker+[0;0;1])/2;
    n_Lm_marker=n_Lm_marker/norm(n_Lm_marker);
end
a=[1; 0; 0];
b=cross(n_Lm_marker,a)/norm(cross(n_Lm_marker,a));
a=cross(b,n_Lm_marker)/norm(cross(b,n_Lm_marker));
RLm2marker=[a b n_Lm_marker];
marker_limit=0.3;
if(norm(fc1(1:2))>marker_limit)
    fc1(1:2)=fc1(1:2)/norm(fc1(1:2))*marker_limit;
end
PLm2marker=[fc1(1:2);0];
% P2e2marker(3)=0;
TLm2marker=[RLm2marker,PLm2marker;0 0 0 1]
Tmarker2Lm=inv(TLm2marker);
Twrite=reshape(Tmarker2Lm,[1 16]);
if(is_write)
fileConfig = fopen('../conf/matlab/Tmarker2Lm.log','w');
fprintf(fileConfig,'%6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f\n',Twrite);
fclose(fileConfig);
fpathResult=['../conf/configurationData/arm',num2str(path),'/Tmarker2Lm.log'];
fileConfig = fopen(fpathResult,'w');
fprintf(fileConfig,'%6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f\n',Twrite);
fclose(fileConfig);
end
trocar_new_vs_old,