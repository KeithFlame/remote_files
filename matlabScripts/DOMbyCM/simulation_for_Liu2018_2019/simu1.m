SL = [100 10 20 15 0 0 0 0.2]';
MP=MultiBackboneParameter_keith;
figure;xlabel("x");ylabel("y");zlabel("z");view([0 0]);

%% FK from psi
% MBP = MP.refreshLso(psi(2));
% psi = [0 160 pi/2 -2 pi/2 -5]';
% qa = Psi2Actuation_keith(psi,SL,MBP);
% [T1,S1] = FKcc_2segs_bending_keith(psi, SL);
% PS_2segs_keith(S1,SL,T1);
% [T,S]=FKco_2segs_bending_keith(qa, MBP);
% PS_2segs_keith(S,SL,T);

%% FK from qa
qa = [0 150 0 0 2 2]';
MBP = MP.refreshLso(qa(2));
psi = Actuation2Psi_keith(qa,SL,MBP);
[T1,S1] = FKcc_2segs_bending_keith(psi, SL);
PS_2segs_keith(S1,SL,T1);
[T,S]=FKco_2segs_bending_keith(qa, MBP);
PS_2segs_keith(S,SL,T);

%% IK 
