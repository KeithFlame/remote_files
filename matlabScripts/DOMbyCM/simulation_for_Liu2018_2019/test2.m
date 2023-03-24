%% base 
options = optimoptions('fmincon','Algorithm','interior-point','Display','iter');
SL = [100 10 20 15 0 0 0 0.2]';
% MP=MultiBackboneParameter_keith;
figure;xlabel("x");ylabel("y");zlabel("z");view([10 20]);
psi_limit = [0.1 6 0.1 0.1 0.1 0.1]';
err = 1e-3;k = 1;
psi0 = [0 60 0 0 0 0]';
%% target and error
Target = eye(4);Target(1:3,4)=[-40 40 100]';
psi = psi0;
x = zeros(6,1);
xT = tem_T2x(Target);
dZt = getdX(x,xT);dZ = dZt;
err_c = max(abs(dZt));
err_b1 = err_c;
%% broyden method
J = eye(6,6);
f=@(x)(norm(J*x-dZt));
[dPsi_1,y,flag] = fmincon(f,x,[],[],[],[],[],[],[],options);

dPsi = scalePsi(dPsi_1,0.1*psi_limit);
iter = 1;
J_1 = J';

while (1)
    dZ_1 = dZ;
    psi = psi - dPsi;
    [T,S] = FKcc_2segs_bending_keith(psi, SL);
    PS_2segs_keith(S,SL,T);
    x = tem_T2x(T);
    dZt = getdX(x,xT);
%     dZ = getdX(x,xT);
    
%     J = Jacobian_Broyden(J,psi0-psi,dZt);  % 当前psi与初始psi0负差值，当前位姿与目标位姿差值200次迭代后误差  +++ 14.9989
%     J = Jacobian_Broyden(J,psi-psi0,dZt);  % 当前psi与初始psi0差值，当前位姿与目标位姿差值200次迭代后误差 +++ 110.0892
%     J = Jacobian_Broyden(J,psi-psi0,-dZt); % 当前psi与初始psi0差值，当前位姿与目标位姿负差值200次迭代后误差 +++14.9989
%     J = Jacobian_Broyden(J,psi0-psi,-dZt); % 当前psi与初始psi0负差值，当前位姿与目标位姿负差值200次迭代后误差 +++110.0892
% 
%     J = Jacobian_Broyden(J,psi0-psi,dZ);  % 当前psi与初始psi0负差值，当前位姿与上一步位姿差值200次迭代后误差 +++12.XXXX
%     J = Jacobian_Broyden(J,psi-psi0,dZ);  % 当前psi与初始psi0差值，当前位姿与上一步位姿差值200次迭代后误差 +++92.5557
%     J = Jacobian_Broyden(J,psi-psi0,-dZ); % 当前psi与初始psi0差值，当前位姿与上一步位姿负差值200次迭代后误差 +++12.8858
%     J = Jacobian_Broyden(J,psi0-psi,-dZ); % 当前psi与初始psi0负差值，当前位姿与上一步位姿负差值200次迭代后误差 +++92.5557
% 
%     J = Jacobian_Broyden(J,-dPsi,dZ);  % 当前psi与上一步psi负差值，当前位姿与上一步位姿差值200次迭代后误差 +++11.0829
%     J = Jacobian_Broyden(J,dPsi,dZ);   % 当前psi与上一步psi差值，当前位姿与上一步位姿差值200次迭代后误差 +++83.7809
%     J = Jacobian_Broyden(J,dPsi,-dZ);  % 当前psi与上一步psi差值，当前位姿与上一步位姿负差值200次迭代后误差 +++11.0829
%     J = Jacobian_Broyden(J,-dPsi,-dZ); % 当前psi与上一步psi负差值，当前位姿与上一步位姿负差值200次迭代后误差 +++83.7809
% 
%     J = Jacobian_Broyden(J,-dPsi,dZt);  % 当前psi与上一步psi负差值，当前位姿与目标位姿差值 200次迭代后误差 +++14.1222 震荡
%     J = Jacobian_Broyden(J,dPsi,dZt);   % 当前psi与上一步psi差值，当前位姿与目标位姿差值 200次迭代后误差 +++74.9182
%     J = CorGrant(J,dPsi,dZ);
    J = Jacobian_Broyden(J,-dPsi,dZ);
%     J_1 = Jacobian_Broyden_Inv(J_1,-dPsi,dZ);
    f=@(x)(norm(getdX(J*x,dZt)));
    [dPsi_1,y,flag] = fmincon(f,dPsi,[],[],[],[],[],[],[],options);

%     J_1 = eye(6)/(J'*J+1e-5*eye(6))*J';
%     dPsi_1 = J_1*dZt;

    dPsi = scalePsi(0.01*dPsi_1,psi_limit);

    iter = iter + 1;
    err_c = max(abs(dZt(1:3)))
    err_b1 = [err_b1;err_c];
    if(err_c <1e-1)
        break;x
    end
    if(iter == 500)
        pause;
    end
end











