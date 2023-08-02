% phi L theta1 L1 delta1 Lr theta2 L2 delta2 Lg
config=[0 150 30/57 100 0 10 30/57 20 0 15]';
[T,T1,T2,T3,T4,T5,s1,s2,s3,s4,s5]=FKfunc2(config);
figure;
hold on;
axis equal;
grid on;
plot3(s1(1,:),s1(2,:),s1(3,:),'k',s2(1,:),s2(2,:),s2(3,:),'r',s3(1,:),s3(2,:),s3(3,:),'g',s4(1,:),s4(2,:),s4(3,:),'b',s5(1,:),s5(2,:),s5(3,:),'m','linewidth',2,'LineStyle','-');
