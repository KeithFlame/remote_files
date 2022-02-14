l=100e-3;
theta1=pi/2;
c=0.5e-3;
Lr=10e-3;
L2=20e-3;
k=0.0191;

theta1i=0.01:0.01:2;
sT1i=sin(theta1i);
cT1i=cos(theta1i);

d=c*sT1i./(1-cT1i);
l1i=d.*theta1i./sT1i;
l1o=l-L2-Lr+d-l1i;
theta1o=theta1-theta1i;

f=0.5*k*(theta1o.^2./l1o+theta1i.^2./l1i);

figure;
plot(theta1i,f);
xlabel('theta1i'),ylabel('f'),title('\it\theta_1_i vs f');