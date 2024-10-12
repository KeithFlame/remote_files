iter=load('sh\iter.log');
psi_b=load('sh\psi.log');
flag_1=psi_b(1,1);
flag_2=psi_b(2,1);
flag_3=psi_b(3,1);
SL = [99.2 10 19.4 25 0.1 5 0.6 0 500 0]';
close;
figure(1);hold on;xlabel("x");
ylabel("y");zlabel("z");

psi1=psi_b(1,:);psi1=resetPsi_L1(psi1);
psi2=psi_b(2,:);psi2=resetPsi_L1(psi2);
psi3=psi_b(3,:);psi3=resetPsi_L1(psi3);
if(iter<50)
    ang_cre=-0.01;
else
    ang_cre=-0.0;
end
if(flag_1>10)
    [T1,S] = FKcc_2segs_nobending_keith(psi1,SL(1:4));
    PS_2segs_keith(S,SL,T1);
    
    T11=T1;
    T11(1:3,4)=T1(1:3,4)+ [-0.1 -0. -0.5]';
    T11(1:3,1:3)=eul2rotm([0 0 ang_cre])*T11(1:3,1:3);
    [psi10,flag1]=IKcc_2segs_nobending_keith(T11,SL(1:4));
else
    flag1=1;
    psi10=zeros(6,1);
end

if(flag_2>10)
    [T2,S] = FKcc_2segs_nobending_keith(psi2,SL(1:4));
    PS_2segs_keith(S,SL,T2);
    
    T21=T2;
    T21(1:3,4)=T2(1:3,4)+ [-0.01 -0.8 -0.5]';
    T21(1:3,1:3)=eul2rotm([0 0 ang_cre])*T21(1:3,1:3);
    [psi20,flag2]=IKcc_2segs_nobending_keith(T21,SL(1:4));
else
    flag2=1;
    psi20=zeros(6,1);
end

if(flag_3>10)
    [T3,S] = FKcc_2segs_nobending_keith(psi3,SL(1:4));
    PS_2segs_keith(S,SL,T3);

    T31=T3;
    T31(1:3,4)=T3(1:3,4)+ 0.*[1.5 2.7 -0.7]';
    T31(1:3,1:3)=eul2rotm([0 0 ang_cre])*T31(1:3,1:3);
    [psi30,flag3]=IKcc_2segs_nobending_keith(T31,SL(1:4));
else
    flag3=1;
    psi30=zeros(6,1);
end


if(flag1>0&&flag2>0&&flag2>0)
else
    disp("\nNo solution\n");
    return;
end
[T10,S] = FKcc_2segs_nobending_keith(psi10,SL(1:4));
[T20,~] = FKcc_2segs_nobending_keith(psi20,SL(1:4));
[T30,~] = FKcc_2segs_nobending_keith(psi30,SL(1:4));
psi11=resetPsi_Phi1(psi10);
psi21=resetPsi_Phi1(psi20);
psi31=resetPsi_Phi1(psi30);
psi=[psi11 psi21 psi31]'

iter=iter+1;

% 打开文档文件以进行写入
fileID = fopen('sh\iter.log', 'w');

% 检查文件是否成功打开
if fileID == -1
    error('无法打开文件 iter.log');
end

% 将数据写入文档
fprintf(fileID, '%d\n', iter);

% 关闭文档文件
fclose(fileID);

% 打开文档文件以进行写入
fileID = fopen('sh\psi.log', 'w');

% 检查文件是否成功打开
if fileID == -1
    error('无法打开文件 psi.log');
end

% 将数据写入文档
fprintf(fileID, '%5f %5f %5f %5f %5f %5f\n', psi(1,:));
fprintf(fileID, '%5f %5f %5f %5f %5f %5f\n', psi(2,:));
fprintf(fileID, '%5f %5f %5f %5f %5f %5f\n', psi(3,:));

% 关闭文档文件
fclose(fileID);