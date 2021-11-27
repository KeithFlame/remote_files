%% get (u,v)
function Tt = getT(name)
global Mpixeli;
global A_new;
global TranslationC21;
global poseCam0;
Mpixel=load(['./map/calib_pic_pixel_out_',name,'.log']);
% getAnew;
A_new=load('./map/Anew.log');
t=load('./map/t.log');
TranslationC21=reshape(t,[3 1]);
poseCam0=[
       1   0   0  0
       0   1   0  0
       0   0   1 100];

Tt=zeros(4,4,size(Mpixel,1));
poseRelativeInit = zeros(6,1);
options = optimoptions(@fminunc,'Algorithm','quasi-newton','MaxFunctionEvaluations', 100000);%,'display', 'iter'

for i =1:size(Mpixel,1)
    Mpixeli=Mpixel(i,:);
    [poseRelativeResult, fval,exitflag,output] = fminunc('reprojectionOverallErrorbyPose', poseRelativeInit, options);
    rotRelative = rotRpy(poseRelativeResult(1:3));
    transRelative = [poseRelativeResult(4); poseRelativeResult(5); poseRelativeResult(6)];
    rotLeftNow = poseCam0(:,1:3) * rotRelative;
    transLeftNow = poseCam0(:,4) + transRelative;
    temT=[rotLeftNow,transLeftNow;[0 0 0 1]];
    Tt(:,:,i)=temT; %[-1 0 0 0;0 -1 0 0;0 0 1 0;0 0 0 1]*;
end
end