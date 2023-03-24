% this function is the Kalman Filter algorithm to estimate the Jacobian 
% matrix of a comlex system

%% base knowledge
SL = [100 10 20 15 0 0 0 0.2]';
R= setR;
err = R(1,1)+R(4,4);
dt = 1e-3;
dpsi = [0 -1 0 0 0 0]';
task_limit = [100 100 100  90  90  90]';
psi_limit = 2*[0.02 1.1 0.02 0.02 0.02 0.02]';
psi = [0 50 0 0 0 0]';

J = eye(6);
Jk_1 = reshape(J', [36,1]);
Pk_1 = eye(36);
Qk_1 = zeros(36);
yita = zeros(36,1);
is_Kalman = 0;

Target = eye(4);
Target(1:3,4) = [-40 -40 105]';
figure;hold on; grid on;axis equal;xlabel('x');ylabel('y');zlabel('z');view([45 15])
plotCoord_keith(Target,2);
[T_cur,S] = FKcc_2segs_bending_keith(psi,SL);
PS_2segs_keith(S,SL,T_cur);
dEr = calcDeviationByT(T_cur,Target);
error = norm(dEr);
iter = 1;
k = 1;
%% execute
while(error>err)
    T_last = T_cur;
    psi = psi + dpsi;
    [T_cur,S] = FKcc_2segs_bending_keith(psi,SL);
    plotCoord_keith(T_cur);

    % 加入误差
    rd = rand(6,1);
    X_cur = fromT2X(T_cur)+R*rd;
    T_cur = fromX2T(X_cur);
    %

    dY = calcDeviationByT(T_last,T_cur);
    dEr = calcDeviationByT(T_cur,Target);
    error = norm(dEr);


    
    if(is_Kalman)
            %
        Hk = fromdPsi2Hk(dpsi);
        lamda = getLamda(Hk, Qk_1, Pk_1, dY);
    %     yita = getYitak(dpsi,dY,Jk_1);
        Pk_ = lamda* Pk_1 + Qk_1;
        Jk_ = Jk_1 + yita;
        Kk = getKalmanGain(Pk_, Hk);
        Pk_1 = getErrorCovarianceMatrix(Kk,Pk_,Hk);
        Jk_1 = getUpdateStateEstimate(dY, Kk, Hk, Jk_);
    %     Qk_1 = getQk(Kk,dY);
        JK = reshape(Jk_1, [6 6])';
    else
        dpsi_J = dpsi;
        dpsi_J([1 3 4 5 6]) = dpsi_J([1 3 4 5 6]) *180/pi;
        JK = Jacobian_Broyden(J,dpsi_J,dY);        
        J = JK;
    end

    JK_ = eye(6)/(JK'*JK+1e-6*eye(6))*JK';
    dpsi_1 = JK_*dEr;%/norm(dEr).*task_limit;

    dpsi = scalePsi2(k*dpsi_1, psi_limit);
    
    iter = iter + 1;
    if(mod(iter, 10) == 0)
        PS_2segs_keith(S,SL,T_cur);
        pause(0.1);
    end
end





