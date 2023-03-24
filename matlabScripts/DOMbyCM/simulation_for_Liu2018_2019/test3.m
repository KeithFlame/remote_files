%% base 
options = optimoptions('fmincon','Algorithm','interior-point','Display','iter');
SL = [100 10 20 15 0 0 0 0.2]';
figure;xlabel("x");ylabel("y");zlabel("z");view([10 20]);
psi_limit = [0.1 6 0.1 0.1 0.1 0.1]';
err = 1e-3;k = 1;
psi0 = [0 60 0 0 0 0]';
%% target and error
Target = eye(4);Target(1:3,4)=[1 1 105]';
psi = psi0;
[T,S] = FKcc_2segs_bending_keith(psi, SL);
x = tem_T2x(T);
xT = tem_T2x(Target);
dZt = getdX(x,xT);dZ = dZt;
err_c = max(abs(dZt));
err_b1 = err_c;
%% broyden method
J = eye(6,6);
f=@(x)(norm(getdX(J*x,dZt)));
[dPsi_1,y,flag] = fmincon(f,x,[],[],[],[],[],[],[],options);

dPsi = scalePsi(0.001*dPsi_1,psi_limit);
iter = 1;
J_1 = J';

while (1)
    x_1 = x;
    psi = psi - dPsi;
    [T,S] = FKcc_2segs_bending_keith(psi, SL);
    PS_2segs_keith(S,SL,T);
    x = tem_T2x(T);
    dZt = getdX(x,xT);
    dZ =getdX(x,x_1);

    J = Jacobian_Broyden(J,-dPsi,dZ);
    J_1 = eye(6)/(J'*J+1e-6*eye(6))*J';
    dPsi_1 = J_1*dZt;

    dPsi = scalePsi(0.1*dPsi_1,psi_limit);

    iter = iter + 1;
    err_c = max(abs(dZt))
    err_b1 = [err_b1;err_c];
    if(err_c <err)
        break;x
    end
    if(iter == 500)
        pause;
    end
end











