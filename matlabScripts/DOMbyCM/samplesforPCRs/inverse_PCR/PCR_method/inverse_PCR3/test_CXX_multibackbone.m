MBP = MultiBackboneParameter_keith;

psi = [45 280 18.4 163.612 0 0];
psi([1 3:6]) = psi([1 3:6])*pi/180;
SL=[99.2 10 19.4 15 0 0 0 0.1];
MBP = MBP.resetZeta(0.1);
u = Psi2Curvature_keith(psi,SL);
MBP = MBP.refreshLso(psi(2));
QA = Curvature2Actuation_keith(u,MBP);
qa = [psi(1);psi(2);QA*1000];
Psi2Actuation_keith(psi,SL,MBP);
MBP.dGamma*u*1000