t=6;
theta2in=0.001:0.001:1;
L2in=t*theta2in./(1-cos(theta2in));
figure;hold on;grid on;
plot(theta2in,L2in,'-');
xlabel('theta2in');ylabel('L2in');
