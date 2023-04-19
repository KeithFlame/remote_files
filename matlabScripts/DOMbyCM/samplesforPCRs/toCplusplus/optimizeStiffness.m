function [KSI,guesss] =optimizeStiffness(guess,pose,qa,FMfm,MBPF,ksi)


k_ratio = ones(12,1)*1e-3;
k_ratio([1 5]) = k_ratio([1 5])*1e2;
lambda = 1e-6;
eps = 1e-4;
J = zeros(6,12);
Fe=FMfm(1:3);Me=FMfm(4:6);fe=FMfm(7:9);le=FMfm(10:12);
MBPF(1) = MBPF(1).resetK12(ksi(9:10));
MBPF(2) = MBPF(2).resetK12(ksi(11:12));
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
aksi = ksi;
ksi_ = ksi;
iter = 0;
KSI_ = zeros(102,13);

while(err>eps)

    for i = 1:12
        ksi_(i) = ksi(i)*k_ratio(i)+ksi(i);
        MBPF(1) = MBPF(1).resetK12(ksi_(9:10));
        MBPF(2) = MBPF(2).resetK12(ksi_(11:12));
        [~,~,~,~,~,~,~,~,y3]=shootingOpt_keith(guess,qa,Fe,Me,fe,le,MBPF,ksi_);
        T(1:3,1)=y3(end,4:6)';T(1:3,2)=y3(end,7:9)';
        T(1:3,3)=y3(end,10:12)';T(1:3,4)=y3(end,1:3)';
        T(1:3,1:3) = normalizeRotation(T(1:3,1:3));
        x__ = calcDeviationByT(T,T_);
        x__(4:6) = x__(4:6)/180*pi;
        J(:,i) = x__/k_ratio(i);
        ksi_(i) = ksi(i);
        MBPF(1) = MBPF(1).resetK12(ksi(9:10));
        MBPF(2) = MBPF(2).resetK12(ksi(11:12));
    end
    dT = calcDeviationByT(T_,pose);
    dT(4:6) = dT(4:6)/180*pi;
    ksi = ksi - eye(12)/(J'*J+lambda*eye(12))*J'*(dT).*ksi*1e-1;
    min0 = find(ksi<1e-3);
    if(~isempty(min0))
        ksi(min0) = ones(size(min0,1),1)*1e-3;
    end
    MBPF(1) = MBPF(1).resetK12(ksi(9:10));
    MBPF(2) = MBPF(2).resetK12(ksi(11:12));
    ksi_ = ksi;
    [guess,~,~,~,~,y0,y1,y2,y3]=shootingOpt_keith(guess,qa,Fe,Me,fe,le,MBPF,ksi);
    T_(1:3,1)=y3(end,4:6)';T_(1:3,2)=y3(end,7:9)';
    T_(1:3,3)=y3(end,10:12)';T_(1:3,4)=y3(end,1:3)';
    T_(1:3,1:3) = normalizeRotation(T_(1:3,1:3));
    dT = calcDeviationByT(T_,pose);
    dT(4:6)=dT(4:6)/180*pi/10;
    err = norm(dT);
    errors = [errors err];
    iter = iter +1;
    KSI_(iter,1:12)=ksi';
    KSI_(iter,13)=err;
    if(iter>100)
        break;
    end
    
end

guesss = guess;
KSI = ksi;
end


