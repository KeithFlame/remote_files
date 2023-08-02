L=130e-3;
N=1:50;
a=4.2/0.432;
u=a./N;
figure(1);
plot(N,u);
theta = u*L*180/pi;
figure(2);
plot(N,theta);

%%
F=140;
EA = 172790;
epsilon = F/EA./N;
figure(3);
plot(N,epsilon);