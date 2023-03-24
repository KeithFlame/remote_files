%% base 
options = optimoptions('fmincon','Algorithm','interior-point','Display','iter');
SL = [100 10 20 15 0 0 0 0.2]';
figure;xlabel("x");ylabel("y");zlabel("z");view([10 20]);
psi_limit = [0.1 6 0.1 0.1 0.1 0.1]';
task_limit = [100 100 100  90  90  90]';
err = 1e-2;k = 1e-2;
psi0 = [0 60 1 1 1 1]';
%% target and error
Target = eye(4);Target(1:3,4)=[-40 -40 105]';
psi = psi0;
[T,S] = FKcc_2segs_bending_keith(psi, SL);
PS_2segs_keith(S,SL,T);

T = [    0.0716    0.2748    0.9588   -2.3706
    0.0153   -0.9615    0.2745   20.2131
    0.9973   -0.0050   -0.0730   91.1437
         0         0         0    1.0000];
x = tem_T2x(T);
xT = tem_T2x(Target);
dZt = getdX(x,xT);dZ = dZt;
err_c = max(abs(dZt));
err_b1 = err_c;
%% broyden method
J = eye(6,6);
f=@(x)(x .*[1 pi/180 1 1 1 1]' * 180/pi);
dPsi = 0.1*psi_limit;
iter = 1;
m = 1;
path = "F:\code_git\cppScripts\pro_repo\test_anything\T.log";

while (1)
    x_1 = x;
    psi = psi - dPsi;
    [T,S] = FKcc_2segs_bending_keith(psi, SL);
    plotCoord_keith(T);
    x = tem_T2x(T);
    dZt = getdX(x,xT);
    dZ =getdX(x,x_1);

    J = Jacobian_Broyden(J,f(-dPsi),dZ);
    J_1 = eye(6)/(J'*J+1e-6*eye(6))*J';
    dPsi_1 = J_1*dZt/norm(dZt).*task_limit;

    dPsi = scalePsi(k*dPsi_1,psi_limit);

    iter = iter + 1;
    err_c = max(abs(dZt))
    err_b1 = [err_b1;err_c];
    if(err_c <err)
        break;
    end
    if(mod(iter,10) == 0)
        pause(0.1);
    end
    fpath = fopen(path,'w');
    fprintf(fpath,"%6f    %6f    %6f    %6f    %6f    %6f",x);
    fclose(fpath);
end
PS_2segs_keith(S,SL,T);










