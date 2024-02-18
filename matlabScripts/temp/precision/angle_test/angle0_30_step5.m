T_marker1=[
-0.0582166   0.059562   0.996526    115.337
-0.0441848   0.997087 -0.0621768    30.2218
 -0.997326 -0.0476511 -0.0554153    387.922
         0          0          0          1];
T_marker2=[
  0.030395  0.0664464   0.997327    116.514
-0.0452435   0.996857 -0.0650362    30.2212
 -0.998513 -0.0431457  0.0333057    387.933
         0          0          0          1];
T_marker3=[
  0.115649  0.0729846   0.990605    117.649
-0.0470465    0.99658 -0.0679323    30.2045
 -0.992175 -0.0387482   0.118687     388.04
         0          0          0          1];
T_marker4=[
  0.202278  0.0790832    0.97613    118.806
-0.0483178   0.996326 -0.0707068    30.1715
 -0.978135  -0.032862   0.205356    388.244
         0          0          0          1];
T_marker5=[
  0.284877  0.0844179    0.95484    119.922
 -0.050976   0.996039 -0.0728517     30.158
 -0.957208 -0.0279202   0.288051    388.586
         0          0          0          1];
T_marker6=[
   0.36847  0.0901518   0.925398    120.903
-0.0535919   0.995718 -0.0755714     30.138
 -0.928105 -0.0218178   0.371637    388.962
         0          0          0          1];
T_marker7=[
  0.448964  0.0939143   0.888601    121.938
-0.0560586   0.995463 -0.0768848    30.1027
 -0.891789 -0.0152952   0.452192    389.339
         0          0          0          1];

T21=T_marker1\T_marker2;
T31=T_marker1\T_marker3;
T41=T_marker1\T_marker4;
T51=T_marker1\T_marker5;
T61=T_marker1\T_marker6;
T71=T_marker1\T_marker7;
axang1 = rotm2axang(T_marker1(1:3,1:3));
axang2 = rotm2axang(T_marker2(1:3,1:3));
axang3 = rotm2axang(T_marker3(1:3,1:3));
axang4 = rotm2axang(T_marker4(1:3,1:3));
axang5 = rotm2axang(T_marker5(1:3,1:3));
axang6 = rotm2axang(T_marker6(1:3,1:3));
axang7 = rotm2axang(T_marker7(1:3,1:3));
axang21 = rotm2axang(T21(1:3,1:3));
axang31 = rotm2axang(T31(1:3,1:3));
axang41 = rotm2axang(T41(1:3,1:3));
axang51 = rotm2axang(T51(1:3,1:3));
axang61 = rotm2axang(T61(1:3,1:3));
axang71 = rotm2axang(T71(1:3,1:3));
axang=[axang1; axang2; axang3; axang4; axang5; axang6; axang7];
a1 = [axang1(4) axang2(4) axang3(4) axang4(4) axang5(4) axang6(4) axang7(4)];
a1 = a1 * 180 / pi;
a2 = [axang21(4) axang31(4) axang41(4) axang51(4) axang61(4) axang71(4)];
a2 = a2 * 180 / pi;
err_2 = a2-(1:6)*5;
figure(1);hold on;grid on; axis equal;
% xlabel("x");ylabel("y");xlabel("z");
axis([-0 1 -0 1 -0.5 0.5])
% plot3(axang(:,1),axang(:,2),axang(:,3),'*')
figure(2);
plot(err_2)