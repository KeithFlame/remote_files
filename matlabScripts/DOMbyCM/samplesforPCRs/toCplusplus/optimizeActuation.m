function [QA,guessa] =optimizeActuation(guess,pose,qa,FMfm,MBPF,ksi)


q_ratio = ones(12,1)*1e-4;
lambda = 1e-6;
eps = 1e-3;
J = zeros(6,12);
Fe=FMfm(1:3);Me=FMfm(4:6);fe=FMfm(7:9);le=FMfm(10:12);
MBPF(1) = MBPF(1).refreshLso(qa(2)*1000);
MBPF(2) = MBPF(2).refreshLso(qa(8)*1000);
[guess,~,~,~,~,~,~,~,y3]=shootingOpt_keith(guess,qa,Fe,Me,fe,le,MBPF,ksi);
T = eye(4);
T(1:3,1)=y3(end,4:6)';T(1:3,2)=y3(end,7:9)';
T(1:3,3)=y3(end,10:12)';T(1:3,4)=y3(end,1:3)';
T(1:3,1:3) = normalizeRotation(T(1:3,1:3));
T_ = T;
dT = calcDeviationByT(T_,pose);
dT(4:6)=dT(4:6)/180*pi/10;
err = norm(dT);
errors=err;
aqa = qa;
qa_ = qa;
while(err>eps)

    for i = 1:12
        qa_(i) = q_ratio(i)+qa(i);
        MBPF(1) = MBPF(1).refreshLso(qa_(2)*1000);
        MBPF(2) = MBPF(2).refreshLso(qa_(8)*1000);
        [~,~,~,~,~,~,~,~,y3]=shootingOpt_keith(guess,qa_,Fe,Me,fe,le,MBPF,ksi);
        T(1:3,1)=y3(end,4:6)';T(1:3,2)=y3(end,7:9)';
        T(1:3,3)=y3(end,10:12)';T(1:3,4)=y3(end,1:3)';
        T(1:3,1:3) = normalizeRotation(T(1:3,1:3));
        x__ = calcDeviationByT(T,T_);
        x__(4:6) = x__(4:6)/180*pi;
        J(:,i) = x__/q_ratio(i);
        qa_(i) = qa(i);
        MBPF(1) = MBPF(1).refreshLso(qa(2)*1000);
        MBPF(2) = MBPF(2).refreshLso(qa(8)*1000);
    end
    dT = calcDeviationByT(T_,pose);
    dT(4:6) = dT(4:6)/180*pi;
    qa = qa - eye(12)/(J'*J+lambda*eye(12))*J'*(dT)*5e-2;

    qa_ = qa;
    MBPF(1) = MBPF(1).refreshLso(qa(2)*1000);
    MBPF(2) = MBPF(2).refreshLso(qa(8)*1000);
    [guess,~,~,~,~,y0,y1,y2,y3]=shootingOpt_keith(guess,qa,Fe,Me,fe,le,MBPF,ksi);
    T_(1:3,1)=y3(end,4:6)';T_(1:3,2)=y3(end,7:9)';
    T_(1:3,3)=y3(end,10:12)';T_(1:3,4)=y3(end,1:3)';
    T_(1:3,1:3) = normalizeRotation(T_(1:3,1:3));
    dT = calcDeviationByT(T_,pose);
    dT(4:6)=dT(4:6)/180*pi/10;
    err = norm(dT);
    errors = [errors err];
end
guessa = guess;
QA = qa;
end


