    
poseCam0=[
       1   0   0  0
       0   1   0  0
       0   0   1 100];

poseRelativeInit=zeros(6,1);
options = optimoptions(@fminunc,'Algorithm','quasi-newton','display', 'iter','MaxFunctionEvaluations', 100000);%,'display', 'iter'
[poseRelativeResult, fval,exitflag,output] = fminunc('reprojectionOverallErrorbyPose', poseRelativeInit, options);
rotRelative = rotRpy(poseRelativeResult(1:3));
transRelative = [poseRelativeResult(4); poseRelativeResult(5); poseRelativeResult(6)];
rotLeftNow = poseCam0(:,1:3) * rotRelative;
transLeftNow = poseCam0(:,4) + transRelative;
temT=[rotLeftNow,transLeftNow;[0 0 0 1]];