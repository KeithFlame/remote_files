SL = [120 29.5 19.6 5.71]';
psi = [-0.653735, 101.013199,0.706698,0.233023,1.966693,-2.796794]';
[T,S]=FKcc_2segs_bending_keith(psi,SL);
figure(1);axis equal; grid on;
hold on;
PS_2segs_keith(S,SL,T);