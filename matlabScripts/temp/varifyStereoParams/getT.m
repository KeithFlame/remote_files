%% get (u,v)
% getAnew;
poseCam0 = setTinit;
A_new=[];
TranslationC21=[];
poseRelativeInit = zeros(6,1);
options = optimoptions(@fminunc,'display', 'iter','Algorithm','quasi-newton','MaxFunctionEvaluations', 100000);%
[poseRelativeResult, fval,exitflag,output] = fminunc('reprojectionOverallErrorbyPose', poseRelativeInit, options);
rotRelative = rotRpy(poseRelativeResult(1:3));
transRelative = [poseRelativeResult(4); poseRelativeResult(5); poseRelativeResult(6)];
rotLeftNow = poseCam0(1:3,1:3) * rotRelative;
transLeftNow = poseCam0(:,4) + transRelative;
temT=[rotLeftNow,transLeftNow;[0 0 0 1]];
Tt=temT