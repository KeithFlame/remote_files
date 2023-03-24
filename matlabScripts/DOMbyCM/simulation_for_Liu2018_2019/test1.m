
%% base 
options = optimoptions('fmincon','Algorithm','interior-point','Display','iter');
SL = [100 10 20 15 0 0 0 0.2]';
% MP=MultiBackboneParameter_keith;
figure;xlabel("x");ylabel("y");zlabel("z");view([10 20]);
psi_limit = [0.1 6 0.1 0.1 0.1 0.1]';
err = 1e-3;k = 1;

%% target and error
psi0 = [0 60 0 0 0 0]';
psi = psi0;
% MBP = MP.refreshLso(psi0(2));
% qa = Psi2Actuation_keith(psi0,SL,MBP);
[T0,S] = FKcc_2segs_bending_keith(psi, SL);
x0 = tem_T2x(T0);
PS_2segs_keith(S,SL,T0);
Target = eye(4);Target(1:3,4)=[10 0 100]';
xT = tem_T2x(Target);

dX = x0-xT;
err_c = max(abs(dX));
err_b0 = err_c;

%% 差分法
iter = 1;
while(err_c - err > 0)
    J_ = Jacobian_differential_keith(psi,SL);
    dpsi = J_*dX;
    dpsi = scalePsi(dpsi,psi_limit);
    psi = psi - k*dpsi;
    [T,S] = FKcc_2segs_bending_keith(psi, SL);
    x = tem_T2x(T);
    PS_2segs_keith(S,SL,T);
    dX = x-xT;
    err_c = max(abs(dX));
    iter = iter + 1;
    err_b0 = [err_b0;err_c];
end

%% second target and error
Target = eye(4);Target(1:3,4)=[15 0 101]';
psi0 = psi;
xT = tem_T2x(Target);
dZt = x-xT;
err_c = max(abs(dZt));
err_b1 = err_c;
%% broyden method
J = eye(6,6);
x = zeros(6,1);
f=@(x)(norm(J*x-dZt));
% % % J = [ones(6,1) ones(6,1)*2 ones(6,1)*3 ones(6,1)*4 ones(6,1)*5 ones(6,1)*6];

% % % J__ = reshape(J_,[6 6]);
J_ = reshape(J',[36 1]);
x1 = zeros(36,1);
f1=@(x1)(norm(x1-J_));
[dPsi_1,y,flag] = fmincon(f,x,[],[],[],[],[],[],[],options);

dPsi = scalePsi(dPsi_1,0.1*psi_limit);
iter = 1;
A = zeros(6,36);
while (1)
    dZ_1 = dZt;
    psi = psi + dPsi;
    [T,S] = FKcc_2segs_bending_keith(psi, SL);
    PS_2segs_keith(S,SL,T);
    x = tem_T2x(T);
    dZt = x - xT;
    dZ = dZt-dZ_1;
    
    J = Jacobian_Broyden(J,-psi+psi0,dZt);
% % %     A(1,1:6) = dPsi';A(2,7:12) = dPsi';A(3,13:18) = dPsi';
% % %     A(4,19:24) = dPsi';A(5,25:30) = dPsi';A(6,31:36) = dPsi';
% % %     J_ = reshape(J',[36 1]);
% % %     f1=@(x1)(norm(x1-J_));
% % %     [J__,Jy,flag_] = fmincon(f1,x1,A,dZ,[],[],[],[],[],options);
% % %     J = reshape(J__,[6 6])';
    f=@(x1)(norm(J*x1-dZt));
    [dPsi_1,y,flag] = fmincon(f,dPsi,[],[],[],[],[],[],[],options);
% % %     J_1 = eye(6)/(J'*J+1e-7*eye(6))*J';
% % %     dPsi_1 = J_1*dZt;
    dPsi = scalePsi(dPsi_1,0.1*psi_limit);
%     dPsi',J
    iter = iter + 1;
    err_c = max(abs(dZt))
    err_b1 = [err_b0;err_c];
    if(err_c <1)
        break;
    end
end











