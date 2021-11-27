%% get (u,v)
function [Cf,Tt] = getConfigandT
global Mpixeli;
global A_new;
global TranslationC21;
global poseCam0;
Mpixel=load('./calib_pic_pixel_out.log');
% getAnew;
arm_id_read = load('arm_id.log');
A_new=load(['map/',num2str(arm_id_read),'/A_new.log']);
TranslationC21=load(['map/', num2str(arm_id_read), '/t.log']);
poseCam0=[
       1   0   0  0
       0   1   0  0
       0   0   1 100];
Cf=load('./conf/cfg_list.log');
tem=zeros(size(Cf,1),2);
Cf=[tem Cf*pi/180];
Tt=zeros(4,4,size(Cf,1));
cd autoCali_20210221/newCost/logRead;
poseRelativeInit = zeros(6,1);
options = optimoptions(@fminunc,'Algorithm','quasi-newton','MaxFunctionEvaluations', 100000);%,'display', 'iter'
fileConfig = fopen('./logRead/Config.log','w');
filePsi = fopen('./logRead/psi.log','w');
for i =1:size(Mpixel,1)
    Mpixeli=Mpixel(i,:);
    [poseRelativeResult, fval,exitflag,output] = fminunc('reprojectionOverallErrorbyPose', poseRelativeInit, options);
    rotRelative = rotRpy(poseRelativeResult(1:3));
    transRelative = [poseRelativeResult(4); poseRelativeResult(5); poseRelativeResult(6)];
    rotLeftNow = poseCam0(:,1:3) * rotRelative;
    transLeftNow = poseCam0(:,4) + transRelative;
    temT=[rotLeftNow,transLeftNow;[0 0 0 1]];
    Tt(:,:,i)=temT; %[-1 0 0 0;0 -1 0 0;0 0 1 0;0 0 0 1]*
    fprintf(fileConfig,'%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f\n',temT);
    fprintf(filePsi,'%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f\n',Cf(i,:));
end
fclose(fileConfig);
fclose(filePsi);

cd ../../..;
end