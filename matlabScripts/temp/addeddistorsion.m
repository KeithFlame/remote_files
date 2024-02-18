%% function
%1��cropping��Pz����
%2����Ļ��ȼ��㣨GOOVIS��LUCI��
%3������������ȼ��㡣

%% Pz
Pz=40:0.1:60;
fov = 87.56 * pi / 180; % �ӳ��� Dfov��rad
t_Goovis = 4.07; %mm  ������
Cr = t_Goovis./(2*Pz*tan(fov/2)*16/sqrt(16*16+9*9)); % �ü���С�� �ü�����ռԭͼ�İٷֱ�

% Optimization

delta1 = 0.5;  %0.534
delta2 = 0.5; %0.466
m43 = t_Goovis/(2*tan(fov/2)*16/sqrt(16*16+9*9)); %�м����
c30 = t_Goovis./(2*30*tan(fov/2)*16/sqrt(16*16+9*9)); %Pz=30mm;
c120 = t_Goovis./(2*120*tan(fov/2)*16/sqrt(16*16+9*9)); %Pz=120mm;
 W1 = -m43*log(Pz/120)+c120*Pz-c120*120;
 W2 = -m43*log(Pz/30)+Pz*c30-c30*30;
 W = delta1*W1+delta2*W2;
%  plot(Pz,W);

%% goovis ��ʾ���� Goovis �ӳ���Ϊ53.0�ȡ�
I = 64.0;
d = 64.0;
c50 = t_Goovis/(2*30*tan(fov/2)*16/sqrt(16*16+9*9)); % 50mm �Ĳü�����
d_Goovis = sqrt(1920*1920+1080*1080)/2/tan(26.5651/180*pi); %Goovis �ӳ���
dv_Goovis = I*d_Goovis/c50/1920;
Wp_Goovis = dv_Goovis*16/sqrt(16*16+9*9)*2*tan(26.5651/180*pi);

D_serial_Goovis = [dv_Goovis-100,dv_Goovis-50,dv_Goovis,dv_Goovis+50,dv_Goovis+100];
du_Goovis = ((D_serial_Goovis-dv_Goovis)./D_serial_Goovis*I-d)/Wp_Goovis*1920;

%% ����Ļ�ߴ�仯ʱ����ʼ�������� Goovis
%��ͼ������ϵ������ 
Ratio_Goovis = (1-c50)*1920/1080;
% Ratio_Goovis = 4/3;
t_Goovis = (1920-1080*Ratio_Goovis-c50*1920)/4;
% t=0;
t_3d_Goovis = Ratio_Goovis*1080/8+t_Goovis+c50*960-53;
p_base_Goovis = [10+t_Goovis,905-t_Goovis,970+t_Goovis,1865-t_Goovis, t_3d_Goovis,960+t_3d_Goovis];
P_Goovis = zeros(5,6);
for i=1:5
    P_Goovis(i,:)=[p_base_Goovis(1)-du_Goovis(i)/2, p_base_Goovis(2),p_base_Goovis(3),p_base_Goovis(4)+du_Goovis(i)/2,p_base_Goovis(5)-du_Goovis(i)/4,p_base_Goovis(6)+du_Goovis(i)/4];
end
P_Goovis = round(P_Goovis);

%% luci ��ʾ���� Luci���ӳ���Ϊ70.0�ȡ�
d_Luci = sqrt(1920*1920+1080*1080)/2/tan(35/180*pi); %Luci �ӳ���
dv_Luci = I*d_Luci/c50/1920;
Wp_Luci = dv_Luci*16/sqrt(16*16+9*9)*2*tan(35/180*pi);
D_serial_Luci = [dv_Luci-100,dv_Luci-50,dv_Luci,dv_Luci+50,dv_Luci+100];
du_Luci = ((D_serial_Luci-dv_Luci)./D_serial_Luci*I-d)/Wp_Luci*1920;


%% ����Ļ�ߴ�仯ʱ����ʼ�������� Luci

Ratio_Luci = (1-c50)*1920/1080;
% Ratio_Luci = 4/3;
Ratio_Luci=Ratio_Goovis;
t_Luci = (1920-1080*Ratio_Luci-c50*1920)/2;
% t=0;
t_3d_Luci = Ratio_Luci*1080/4+t_Luci+c50-108;

p_base_Luci = [20+t_Luci,1810-t_Luci,1940+t_Luci,3730-t_Luci,t_3d_Luci,1920+t_3d_Luci];
P_Luci = zeros(5,6);
for i=1:5
    P_Luci(i,:)=[p_base_Luci(1)-du_Luci(i), p_base_Luci(2),p_base_Luci(3),p_base_Luci(4)+du_Luci(i),p_base_Luci(5)-du_Luci(i)/2,p_base_Luci(6)+du_Luci(i)/2];
end
P_Luci = round(P_Luci);


%%
fov1 = 87.56*pi/180;
h = tan(fov1/2);
m43 = sqrt(12*12+9*9)/sqrt(16*16+9*9);
m32 = sqrt(13.5*13.5+9*9)/sqrt(16*16+9*9);
mmax = sqrt((16*(1-c50))^2+9*9)/sqrt(16*16+9*9);
fov43 = atan(m43*h)*180/pi*2;
fov32 = atan(m32*h)*180/pi*2;
fovmax =  atan(mmax*h)*180/pi*2;

