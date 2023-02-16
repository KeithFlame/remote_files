
options = optimoptions('fmincon','Algorithm','interior-point','Display','iter');
SL = [100 10 20 15 0 0 0 0.2]';
MP=MultiBackboneParameter_keith;
figure;xlabel("x");ylabel("y");zlabel("z");view([0 0]);
psi_limit = [0.1 1 0.1 0.1 0.1 0.1]';
k = 1e-3;
errp = 1e-3;
erro = 1e-3;
err = [errp;erro];
step = 1;

psi = [0 60 0 0 0 0]';
MBP = MP.refreshLso(psi(2));
qa = Psi2Actuation_keith(psi,SL,MBP);
[T,S]=FKco_2segs_bending_keith(qa, MBP);
PS_2segs_keith(S,SL,T);
J=[eye(6)];
Target = eye(4);Target(1:3,4)=[60 0 100]';

dP = Target(1:3,4)-T(1:3,4);
dr = Target(1:3,1:3)'*T(1:3,1:3);
dR = rotm2eul(dr);
dZ = [dP;dR'];
dPsi_1 = k*(J\dZ);
tp = psi_limit - abs(dPsi_1);
tt = find(tp<0);
dPsi = dPsi_1;
dPsi(tt) = psi_limit(tt).*sign(dPsi_1(tt));
iter = 1;
while (1)
    psi = psi - dPsi;
%     MBP = MP.refreshLso(psi(2));
%     qa = Psi2Actuation_keith(psi,SL,MBP);
%     [T,S]=FKco_2segs_bending_keith(qa, MBP);
    [T,S] = FKcc_2segs_bending_keith(psi, SL);
    PS_2segs_keith(S,SL,T);
    dP = Target(1:3,4)-T(1:3,4);
    dr = Target(1:3,1:3)'*T(1:3,1:3);
    dR = rotm2eul(dr)*180/pi;
    dZ = [dP;dR'];
    J = Jacobian_Broyden(J,dPsi_1,dZ);
    x = zeros(6,1);
    f=@(x)(norm(J*x-dZ));
    [dPsi_1,y,flag] = fmincon(f,dPsi_1,[],[],[],[],[],[],[],options);
    tp = psi_limit - abs(dPsi_1);
    tt = find(tp<0);
    dPsi = dPsi_1;
    dPsi(tt) = psi_limit(tt).*sign(dPsi_1(tt));
    iter = iter + 1;

end











