x= [100 10 20 15 0.1 5 1 600 0 0];
psi = [40 0 90 0 0 0]*pi/180;
u = fromPsi2Curvature(psi,x);
q = tem_fromCurvature2Movitation(u,x);
[qa,MP] = fC2M_2(u,x);
q,qa,