l=100e-3;
theta1=pi/2;
c=0.5e-3;
Lr=10e-3;
L2=20e-3;
L1=100e-3;
k1=0.0191;
zeta=0.1;
ks=k1/zeta;

thetasi=0.001:0.001:0.06;
sTsi=sin(thetasi);
cTsi=cos(thetasi);

d=c*sTsi./(1-cTsi);
lsi=d.*thetasi./sTsi;
lso=l-L1-L2-Lr+d-lsi;
thetaso=(theta1-thetasi).*lso*zeta./(lso*zeta+L1);
theta1o=theta1-thetaso-thetasi;

f=0.5*k1*(theta1o.^2./L1)+0.5*ks*(thetasi.^2./lsi+thetaso.^2./lso);

figure;
plot(thetasi,f);
xlabel('thetasi'),ylabel('f'),title('\it\theta_s_i vs f');