%% base 
options = optimoptions('fmincon','Algorithm','interior-point','Display','iter');
SL = [100 10 20 15 0 0 0 0.2]';
MP=MultiBackboneParameter_keith;
figure;xlabel("x");ylabel("y");zlabel("z");view([10 20]);
q_limit = 1*[0.01 1 0.1 0.1 0.1 0.1]';
tar_limit = [100 100 100 90 90 90]';
err = 1e-3;k = 2e-2;
psi0 = [0 60 0 0 0 0]';
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
plot3(P(1,:), P(2,:), P(3,:), "r*");

psi = psi0;
MBP = MP.refreshLso(psi0(2));
qa = Psi2Actuation_keith(psi0,SL,MBP);
FM = zeros(12,1);
FM(2) = 0;
[T,S] = FKco_2segs_bending_keith(qa,MBP,FM);
PS_2segs_keith(S,SL,T);
x = tem_T2x(T);
xT = tem_T2x(Target);
dZt = getdX(x,xT);dZ = dZt;
err_c = max(abs(dZt));
err_b1 = err_c;
iter_num = zeros(array_capacity,1);
val = zeros(array_capacity,1);
%% broyden method
J = eye(6,6);
dq = 0.1*q_limit;
iter = 1;
m = 1;
noise = 0.5;
for i = 1:array_capacity
    iter = 1;

    Target(1:3,4) = P(:,i);
    xT = tem_T2x(Target);
    dZt = getdX(x,xT);dZ = dZt;
    err_c = norm(dZt);
    while (1)
        x_1 = x;
        qa = qa - dq;
        MBP = MBP.refreshLso(qa(2));
        [T,S] = FKco_2segs_bending_keith(qa, MBP,FM);
        plotCoord_keith(T);
        x = tem_T2x(T,noise);
        dZt = getdX(x,xT);
        err_p = dZt;
        dZ =getdX(x,x_1);
    
        J = Jacobian_Broyden(J,-dq,dZ);
        J_1 = eye(6)/(J'*J+1e-6*eye(6))*J';
        dZ_lim = norm(dZt);
    
        dq_1 = J_1*(dZt/dZ_lim.*tar_limit);
    
        dq = scaleQ(k*dq_1,q_limit);
    
        iter = iter + 1;
        err_c = max(abs(err_p));
        if(err_c <noise+err)
            val(i) = err_c;
            iter_num(i) = iter;
            break;
        end

    end
    PS_2segs_keith(S,SL,T);
    pause(0.1);
end