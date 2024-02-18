function [Qc,flag,iter,Jac] = invKine_7DoF_inEnd(T_base,Ttar,arm)
%INVKINE_7DOF inverse kinematics for a 7—DoF robot
% input1: robot origin (T_base)
% input2: target pose (Ttar)
% input3: robot parameter (arm)
%
% output1: robot acutuation (Qc)
% output2: convergence symbol (flag)
% output3: iteration (iter)
% output4: Jacobian (iter)

Q = zeros(7,1);
lambda = 1e-6;
thre = 1e-4;
d_size = 1e-6;
err = 10;
iter_max = 500;
pair3(err);
errs = err;

[~,~,~,~,~,~,~,~,~,~,~,~,~,~,Tbase7,~]=FK_7DOF_Offset(arm,T_base,Q);
Tcur = Tbase7\Ttar;
Rsd = calcDeviationByT(eye(4), Tcur);
dGuess = eye(7)*d_size;
Jac = zeros(6,7);
iter = 0;
while(err>thre)
    for i = 1:7
        [~,~,~,~,~,~,~,~,~,~,~,~,~,~,Tbase7_,~]=FK_7DOF_Offset(arm,T_base,Q+dGuess(:,i));
        Tcur_ = Tbase7_\Ttar;
        Jac(:,i) = calcDeviationByT(Tcur_, Tcur)/d_size;
    end
    dQ = eye(7)/(Jac'*Jac+lambda*eye(7))*Jac'*Rsd;
    max_dQ = max(abs(dQ));
    if(max_dQ>pi)
        dQ = dQ/max_dQ*pi;
    end
    Q=Q+dQ;
    Q = checkQ(Q);
    [~,~,~,~,~,~,~,~,~,~,~,~,~,~,Tbase7,~]=FK_7DOF_Offset(arm,T_base,Q);
    Tcur = Tbase7\Ttar;
    Rsd = calcDeviationByT(eye(4), Tcur);
    err = norm(Rsd);
    cter2 = pair3(err);
    cter = abs(cter2(1)+cter2(2)-2*cter2(3))*10;
    if(cter<thre)
        flag = 2;
        Qc=zeros(7,1);
        return;
    end
    if(iter>iter_max)
        flag = 0;
        Qc=zeros(7,1);
        return;
    end
    iter = iter + 1;
    errs=[errs;err];
end
Qc = Q;
flag = 1;
end

function p = pair3(a)
    persistent T;
    if(isempty(T))
        T = zeros(3,1);
    end
    T(3)=T(2);
    T(2)=T(1);
    T(1)=a;
    p=T;
end
function Qt = checkQ(Qc)
    Q = Qc;
    if(Qc(1)>pi)
        Q(1) = Q(1)-2*pi;
    elseif(Qc(1)<-pi)
        Q(1) = Q(1)+2*pi;
    end
    if(abs(Qc(2))>140/180*pi)
        Q(2) = 140/180*pi*sign(Qc(2));
    elseif(abs(Qc(2))<5/180*pi)
        Q(2) = 5/180*pi*sign(Qc(2));
    end
    if(Qc(3)>pi)
        Q(3) = Q(3)-2*pi;
    elseif(Qc(3)<-pi)
        Q(3) = Q(3)+2*pi;
    end
    if(Qc(4)>0)
        Q(4) = 0;
    elseif(Qc(4)<-pi)
        Q(4) = -pi;
    end
    if(Qc(5)>pi)
        Q(5) = Q(5)-2*pi;
    elseif(Qc(5)<-pi)
        Q(5) = Q(5)+2*pi;
    end
    if(Qc(6)>pi)
        Q(6) = pi;
    elseif(Qc(6)<40/180*pi)
        Q(6) = 40/180*pi;
    end
    if(Qc(7)>pi)
        Q(7) = Q(7)-2*pi;
    elseif(Qc(7)<-pi)
        Q(7) = Q(7)+2*pi;
    end
    Qt = Q;
end