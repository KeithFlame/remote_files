iter=load('sh\iter.log');
psi_b=load('sh\psi.log');
flag_1=psi_b(1,1);
flag_2=psi_b(2,1);
flag_3=psi_b(3,1);
SL = [99.2 10 19.4 25 0.1 5 0.6 0 500 0]';
Trocar1 = [eul2rotm([0 -pi/10 0]) [0 0 0]';[0 0 0 1]];
Trocar2 = [-0.998068668358927	0.00161384024850361	-0.0620993458894009	-18.8669744644419
0.0581165442468079	-0.328822021118264	-0.942602008120346	111.530861587751
-0.0219408414844999	-0.944390530420203	0.328093165926387	47.6394032611166
0	0	0	1];
% Trocar2 = eye(4);
Trocar4 = [-0.884219517998944	0.466526492084466	-0.0225582839125016	-11.5753101316165
-0.142998833897040	-0.316375151110297	-0.937794272356162	116.710943811633
-0.444642752660831	-0.825990191190730	0.346457827971745	46.7689353401736
0	0	0	1];
close;
figure(1);hold on;xlabel("x");
ylabel("y");zlabel("z");
view([0 -90])
plotCoord_keith(eye(4),20,3);

psi1=psi_b(1,:);psi1=resetPsi_L1(psi1);
psi2=psi_b(2,:);psi2=resetPsi_L1(psi2);
psi3=psi_b(3,:);psi3=resetPsi_L1(psi3);
if(iter<50)
    ang_cre=0.01;
else
    ang_cre=0.01;
end
if(flag_1>10)
    [T1,S] = FKcc_2segs_nobending_keith(psi1,SL(1:4));
    S = getS(Trocar1,S');
    PS_2segs_keith(S',SL,T1);
    
    T11=T1;
    T11(1:3,4)=T1(1:3,4)+ [-0 -0. -0]';
    T11(1:3,1:3)=eul2rotm([0 0 ang_cre])*T11(1:3,1:3);
    [psi10,flag1]=IKcc_2segs_nobending_keith(T11,SL(1:4));
else
    flag1=1;
    psi10=zeros(6,1);
end

if(flag_2>10)
    [T2,S] = FKcc_2segs_nobending_keith(psi2,SL(1:4));
    S = getS(Trocar2,S');
    PS_2segs_keith(S',SL,Trocar2*T2);
    plotCoord_keith(Trocar2,10,2);
    plotCoord_keith(Trocar2*T2,10,2);
    
    T21=Trocar2*T2;
    T21(1:3,4)=T21(1:3,4)+ [0.58 -0. -1.2]';
    T21(1:3,1:3)=eul2rotm([-0.0 0.0 ang_cre])*T21(1:3,1:3);
    T22 = Trocar2\T21;
    [psi20,flag2]=IKcc_2segs_nobending_keith(T22,SL(1:4));
else
    flag2=1;
    psi20=zeros(6,1);
end

if(flag_3>10)
    [T3,S] = FKcc_2segs_nobending_keith(psi3,SL(1:4));
    S = getS(Trocar4,S');
    PS_2segs_keith(S',SL,Trocar4*T3);
    plotCoord_keith(Trocar4,10,2);
    plotCoord_keith(Trocar4*T3,10,2);

    T31=Trocar4*T3;
    T31(1:3,4)=T31(1:3,4)+ [0.78 -0. -1.2]';
    T31(1:3,1:3)=eul2rotm([-0.0 0.0 ang_cre])*T31(1:3,1:3);
    T32 = Trocar4\T31;
    [psi30,flag3]=IKcc_2segs_nobending_keith(T32,SL(1:4));
else
    flag3=1;
    psi30=zeros(6,1);
end

% axis([-100 100 -100 100 -10 190])
axis([-100 100 -100 200 -10 150])

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

function S1 = getS(T,S0)

length = size(S0,2);
S1 = zeros(4,length);
S0(4,:)=ones(1,length);
S2 = zeros(3,length);
for i = 1:length
    S1(:,i) = T*S0(:,i);
    S2(:,i) = S1(1:3,i);
end
end
