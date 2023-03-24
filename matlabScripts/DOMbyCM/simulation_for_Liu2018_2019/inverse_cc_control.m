%% base 
SL = [100 10 20 15 0 0 0 0.2]';
figure;hold on;view([10 20]);axis equal; grid on;
xlabel("x");ylabel("y");zlabel("z");
psi_limit = [0.1 6 0.1 0.1 0.1 0.1]';
err = 1e-2;k = 0.01;
psi = [0 55 0 0 0 0]';
%% target and error
Target = eye(4);
array_size = 50;
array_capacity = 6 * array_size;
length = 100;
width = 50;
hight = 100;
P1 = [zeros(1, array_size);linspace(-1,-length/2, array_size);ones(1,array_size)*hight];
P2 = [linspace(1,width, array_size);-length/2*ones(1, array_size);ones(1,array_size)*hight];
P3 = [width*ones(1, 2 * array_size);linspace(-length/2+1,length/2, 2 * array_size);
    ones(1, 2 * array_size)*hight];
P4 = [linspace(width - 1,0, array_size);length/2*ones(1, array_size);ones(1,array_size)*hight];
P5 = [zeros(1, array_size);linspace(length/2-1, 0, array_size);ones(1,array_size)*hight];
P = [P1 P2 P3 P4 P5];
plot3(P(1,:), P(2,:), P(3,:), "r*")
[T,S] = FKcc_2segs_bending_keith(psi, SL);
PS_2segs_keith(S,SL,T);
x = tem_T2x(T);
iter_num = zeros(array_capacity,1);
val = zeros(array_capacity,1);
%% broyden method
J = eye(6,6);
f=@(x)(x .*[1 pi/180 1 1 1 1]' * 180/pi);
dPsi = k*psi_limit;
for i = 1:array_capacity
    iter = 1;

    Target(1:3,4) = P(:,i);
    xT = tem_T2x(Target);
    dZt = getdX(x,xT);dZ = dZt;
    err_c = norm(dZt);
    m = 1;
    while (1)
        x_1 = x;
        psi = psi - dPsi;
        [T,S] = FKcc_2segs_bending_keith(psi, SL);
        PS_2segs_keith(S,SL,T);
        x = tem_T2x(T);
        dZt = getdX(x,xT);
        dZ =getdX(x,x_1);
    
        J = Jacobian_Broyden(J,f(-dPsi),dZ);
        J_1 = eye(6)/(J'*J+1e-6*eye(6))*J';
        dPsi_1 = J_1*dZt;
        t1 = dZt(1:3)/norm(dZt(1:3));
        t2 = dZ(1:3)/norm(dZ(1:3));
        if(t1'*t2<-0.85)
            m = 80;
        else
            m = 1;
        end
        dPsi = scalePsi(m*k*dPsi_1,psi_limit);
    
        iter = iter + 1;
        err_c = norm(dZt);
        if(err_c <err || iter >500)
            val(i) = err_c;
            iter_num(i) = iter;
            break;
        end
    end
    pause(0.1);
end