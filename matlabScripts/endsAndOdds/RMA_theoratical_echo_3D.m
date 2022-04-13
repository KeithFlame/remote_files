%Range Migration Algorithm for 3D  RMA, echo is definded by wavenumber and range
%wu gao yang 2022.3.28
clear ;
clc;
% % close all;
% c = 3E8; %0(m/s) speed of light
%radar parameters
fc = 77E9; %(Hz) center radar frequency
B =3E9; %(hz) bandwidth
Rs =0; %(m) y coordinate to scene center (down range)
Ya = 0; %(m) beginning of new aperture length
L = 0.1; %(m) aperture length
delta_x=0.02/20;L_size=L/delta_x+1;%笨啊 步长lamda/4!
Xa = linspace(-L/2, L/2, L_size);%(m) cross range position of
Za=linspace(-L/2, L/2, L_size);%(m) cross range position of
%radar on aperture L
% Za = 0;
% Ya = Rs; %THIS IS VERY IMPORTANT, SEE GEOMETRY FIGURE 10.6
fsteps =800;

%create SAR if data according to eq 10.4 and 10.5 (mocomp
% to a line) ignoring RVP term
%target parameters, 3 targets

targ_num_down=2;
targ_num_cross=5;
targ_num=targ_num_cross*targ_num_down;
target_pos_1=zeros(targ_num,3);
R=1;
for ii=1:targ_num_cross 
    for jj=1:targ_num_down
        target_pos_1((ii-1)*targ_num_down+jj,:)=[0+0.2*ii,R,0+jj*0.2];
    end
end
 figure;scatter( target_pos_1(:,1),target_pos_1(:,3));title('target');axis equal;
at=1*ones(1,targ_num);
radar_pos=[0,Rs,0];
tic
Echo =create_sar_echo(radar_pos,target_pos_1,Xa,Za,fsteps,fc,B,at,targ_num);
disp('create echo time:');toc 
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   RMA

Nx = size(Xa,2);                            % The sampling points in the horizontal direction
Nz = size(Za,2);                            % The sampling points in the vertical direction

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% System parameters
dx = delta_x;                               % Sampling distance at x (horizontal) axis in mm
dy = delta_x;                               % Sampling distance at y (vertical) axis in mm
nFFTspace = fsteps*15;                      % Number of FFT points for Spatial-FFT
c = physconst('lightspeed');
F0 = (77)*1e9;
k = 2*pi*F0/c;
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RMA Imaging
nFFTtime = fsteps;
tic
Sr = fft(Echo,nFFTtime,3);                               % Range FFT
clear Echo;
figure;imagesc(abs(reshape(Sr,[],nFFTtime)))
M=zeros(length(Xa)*length(Za));ID=zeros(length(Xa)*length(Za));
for ii=1:length(Xa)
    for jj=1:length(Za)
      [ M(ii,jj) ,ID(ii,jj)] = max(abs(Sr(ii,jj,:)));
    end
end
ID_select=round(sum(sum(ID))/length(Xa)/length(Za));                                        
sarData = squeeze(Sr(:,:,ID_select)).';                  % Selecting echo data after pulse compression  这实际是选择在指定距离平面的回波
R_middle = (3E8*(fsteps-ID_select)/(2*B));
disp('3D Range profile time:');toc 
imSize = 6000;                                            % Size of 2D image area in mm
tic
wSx = 2*pi/(dx);                                    % Sampling frequency for Target Domain
kX = linspace(-(wSx/2),(wSx/2),nFFTspace);               % kX-Domain
wSy = 2*pi/(dy);                                    % Sampling frequency for Target Domain
kY = (linspace(-(wSy/2),(wSy/2),nFFTspace)).';           % kY-Domain
K = single(sqrt((2*k).^2 - kX.^2 - kY.^2));
phaseFactor0 = exp(-1i*R *K);                        %此处花费时间至少10s
clear K;
phaseFactor0((kX.^2 + kY.^2) > (2*k).^2) = 0;%为什么会出现波数小于其他维度 ，小于的是易消散项 会随成像中心到孔径面的距离快速消散掉
% phaseFactor = K.*phaseFactor0;%为什么又乘一遍
% phaseFactor=phaseFactor0;clear phaseFactor0;
phaseFactor0 = fftshift(fftshift(phaseFactor0,1),2);
disp('filter  time:');toc 
% Padding matrix with 0 目的是
tic
[yPointM,xPointM] = size(sarData);
[yPointF,xPointF] = size(phaseFactor0);
if (xPointF > xPointM)
    sarData = padarray(sarData,[0 floor((xPointF-xPointM)/2)],0,'pre');
    sarData = padarray(sarData,[0 ceil((xPointF-xPointM)/2)],0,'post');
else
    phaseFactor0 = padarray(phaseFactor0,[0 floor((xPointM-xPointF)/2)],0,'pre');
    phaseFactor0 = padarray(phaseFactor0,[0 ceil((xPointM-xPointF)/2)],0,'post');
end

if (yPointF > yPointM)
    sarData = padarray(sarData,[floor((yPointF-yPointM)/2) 0],0,'pre');
    sarData = padarray(sarData,[ceil((yPointF-yPointM)/2) 0],0,'post');
else
    phaseFactor0 = padarray(phaseFactor0,[floor((yPointM-yPointF)/2) 0],0,'pre');
    phaseFactor0 = padarray(phaseFactor0,[ceil((yPointM-yPointF)/2) 0],0,'post');
end
disp(' padding time:');toc 
tic
sarDataFFT = fft2(sarData,nFFTspace,nFFTspace);
sarImage_2DRMA = ifft2(sarDataFFT.*phaseFactor0);%看似这个系数引入的虚部不影响积分 实际里面有0  最耗时的就是这两步
% sarImage_2DRMA = phaseFactor.*ifft2(sarDataFFT);%看似这个系数引入的虚部不影响积分 实际里面有0
clear sarData sarDataFFT ID phaseFactor0;
[yPointT,xPointT] = size(sarImage_2DRMA);
xRangeT = dx * (-(xPointT-1)/2 : (xPointT-1)/2);
yRangeT = dy * (-(yPointT-1)/2 : (yPointT-1)/2);
indXpartT = xRangeT>(-imSize/2) & xRangeT<(imSize/2);
indYpartT = yRangeT>(-imSize/2) & yRangeT<(imSize/2);
xRangeT = xRangeT(indXpartT);
yRangeT = yRangeT(indYpartT);
sarImage_2DFFT1 = abs(sarImage_2DRMA(indYpartT,indXpartT));
sarImage_2DFFT1 = flipud(rot90(sarImage_2DFFT1,-1));%现在横轴是downrange 顺时针转90就是cross range 再cross 倒序与坐标对应
% sarImage_2DFFT1 =flipud(fliplr(sarImage_2DFFT1)) ;%现在横轴是downrange 顺时针转90就是cross range 再cross 倒序与坐标对应
disp('2DIFFT time:');toc 
tic
figure;imagesc(xRangeT,yRangeT,2*db(sarImage_2DFFT1/max(sarImage_2DFFT1(:))),[-50,0]);
axis  xy;
axis equal;colormap('jet');
title 'Sar Image-2D RMA';cbar = colorbar;
set(get(cbar, 'Title'), 'String','dB','fontsize',13);
xlabel('X-axis');ylabel('Z-axis');
% figure;imagesc(abs(sarImage_2DFFT1));
% axis equal xy ;colormap('jet');
% title 'Sar Image-2D RMA';cbar = colorbar;
disp('picture time:');toc 

function[sif]=create_sar_echo(radar_pos,target_pos,Xa,Za,fsteps,fc,B,at,N)
%Rt and Rb for 3 targets according to equation 10.26
c = 3E8; %0(m/s) speed of light
Rs=radar_pos(1);
Rb=zeros(N,1);
Rt=zeros(length(Za),length(Xa),N);
for ii=1:N
    Rb(ii)=sqrt((target_pos(ii,2)-radar_pos(2))^2);
    Rt_Ya=sqrt((Xa-target_pos(ii,1) ).^2 + Rb(ii)^2);
    for jj=1:length(Za)
    Rt(jj,:,ii)=sqrt((Za(jj)-target_pos(ii,3) ).^2 +  Rt_Ya.^2);
    end
end


Kr = linspace(((4*pi/c)*(fc - B/2)), ((4*pi/c)*(fc + B/2)), fsteps);
setATKRRT(at,Kr,Rt);
%according to range defined on bottom of page 410
phi_if=zeros(size(Za,2),size(Xa,2),fsteps );
% phi_if_n=zeros(size(Za,2),size(Xa,2),fsteps );
% tic;
% for nn=1:N 
%     for kk=1:size(Za,2)
%         for ii = 1: fsteps %0step thru each time step to find phi_if
%             for jj = 1:size(Xa,2)  %step thru each azimuth step
%             phi_if_n(kk,jj,ii) =at(nn) .*exp(-1i*Kr(ii)*(Rt(kk,jj,nn) - Rs));
%            end
%         end
%     end
%     phi_if=phi_if+phi_if_n;
% end
% disp('echo serial time:');toc
[At, KR, RT]=setATKRRT;
%% 并行计算 start
Za_len=size(Za,2);Xa_len=size(Xa,2);
tic;
%  p=parpool(4);
 for nn=1:N
     tic;
     xxxx=zeros(1,Za_len*Xa_len*fsteps);
     
    parfor index=1:Za_len*Xa_len*fsteps
        [At, KR, RT]=setATKRRT;
        ii=floor((index-1)/(Za_len*Xa_len))+1;
        jj=floor((mod(index-1,Za_len*Xa_len))/Za_len)+1;
        kk=mod((mod(index-1,Za_len*Xa_len)),Za_len)+1;
        xxxx(index)=At(nn) * exp(-1i*KR(ii)*(RT(kk,jj,nn) - Rs));
    end
    phi_if_n_Vec=xxxx';
    phi_if_n=reshape(phi_if_n_Vec,[Za_len,Xa_len,fsteps]);
    phi_if=phi_if+phi_if_n;
    toc;
 end
%  delete(gcp('nocreate'));
disp('echo par time:');toc;
%% 并行计算 end

sif=phi_if;

end

function [At, KR, RT]=setATKRRT(at,Kr,Rt)

    if(nargin == 0)
    end
    persistent at0;
    persistent kr0;
    persistent rt0;
    if(nargin>0||isempty(at0)||isempty(kr0)||isempty(rt0))
        at0=at;
        kr0=Kr;
        rt0=Rt;
    end
    At=at0;
    KR=kr0;
    RT=rt0;
end