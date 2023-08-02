SL=[100 10 20 15 0.1];
psi=[0 140 40 90 0 0]/180*pi;
psi(2)=psi(2)/pi*180;
T=FKcc_2segs_bending_keith(psi,SL);