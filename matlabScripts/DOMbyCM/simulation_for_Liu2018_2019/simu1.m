SL = [100 10 20 15 0 0 0 0.2]';
MBP=MultiBackboneParameter_keith;
figure;xlabel("x");ylabel("y");zlabel("z");view([45 20]);

%% FK from psi

psi = [0.0000   81.9265   1.3377    0.7854+pi    0.9843    0.7854]';
MBP = MBP.refreshLso(psi(2));
qa = Psi2Actuation_keith(psi,SL,MBP);
[T1,S1] = FKcc_2segs_bending_keith(psi, SL);
PS_2segs_keith(S1,SL,T1);
[T,S]=FKco_2segs_bending_keith(qa, MBP);
PS_2segs_keith(S,SL,T);

%% FK from qa
figure;xlabel("x");ylabel("y");zlabel("z");view([45 20]);
qa = [-0.0671   51.1584    1.2328    0.1205    0.0554    0.0419]';
MBP = MP.refreshLso(qa(2));
psi = Actuation2Psi_keith(qa,SL,MBP);
[T1,S1] = FKcc_2segs_bending_keith(psi, SL);
PS_2segs_keith(S1,SL,T1);
[T,S]=FKco_2segs_bending_keith(qa, MBP);
PS_2segs_keith(S,SL,T);

%% IK 
