function Tt = getjointLimitT
    global Mpixeli;
    global A_new;
    global TranslationC21;
    global poseCam0;
    Mpixel=load('./fat_pic_pixel_out.log');
    num_psi = size(Mpixel,1);
    % getAnew;
    arm_id_read = load('arm_id.log');
    A_new=load(['map/',num2str(arm_id_read),'/A_new.log']);
    TranslationC21=load(['map/', num2str(arm_id_read), '/t.log']);
    poseCam0=[
           1   0   0  0
           0   1   0  0
           0   0   1 100];
    Tt=zeros(4,4,num_psi);
    cd autoCali_20210221/newCost/logRead;
    poseRelativeInit = zeros(6,1);
    options = optimoptions(@fminunc,'Algorithm','quasi-newton','MaxFunctionEvaluations', 100000);%,'display', 'iter'
    for i =1:num_psi
        Mpixeli=Mpixel(i,:);
        [poseRelativeResult, fval,exitflag,output] = fminunc('reprojectionOverallErrorbyPose', poseRelativeInit, options);
        rotRelative = rotRpy(poseRelativeResult(1:3));
        transRelative = [poseRelativeResult(4); poseRelativeResult(5); poseRelativeResult(6)];
        rotLeftNow = poseCam0(:,1:3) * rotRelative;
        transLeftNow = poseCam0(:,4) + transRelative;
        temT=[rotLeftNow,transLeftNow;[0 0 0 1]];
        Tt(:,:,i)=temT; 
    end

    cd ../../..;
end