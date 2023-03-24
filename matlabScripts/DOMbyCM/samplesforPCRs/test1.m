SL = [100 10 20 15 0 0 0 0.2]';
q1 = [0 60 1 1 1 1]';
q2 = [0 60 1 1 1 1]';
MBP1 = MultiBackboneParameter_keith;
MBP2 = MultiBackboneParameter_keith;
MBP1 = MBP1.refreshLso(q1(2));
MBP2 = MBP2.refreshLso(q2(2));
MBPF = [MBP1 MBP2];

psi = Actuation2Psi_keith(q1,SL,MBP1);
T = FKcc_2segs_bending_keith(psi,SL);

T2 = FKco_2segs_bending_keith(q1,MBP1);
% [Tend, S] = FKco_Narms_bending_keith(1, q1, MBP1);
qa = [q1' q2']';
[Tend, S] = FKco_Narms_bending_keith(2, qa, MBPF);