%% ----Posing Manipulators---- %%
%---individual params---%
arm1.id=1;

%关节外径
arm1.W1=100;
arm1.W2=100;
arm1.W3=100;
arm1.W4=100;
arm1.W5=100;
arm1.W6=100;
arm1.W7=80;

%2,4,6关节长度
arm1.H2_1=75;arm1.H2_2=75;
arm1.H4_1=75;arm1.H4_2=75;
arm1.H6_1=75;arm1.H6_2=75;

%DH
arm1.Alpha1=pi;
arm1.a1=0;
arm1.d1=150; %1

arm1.Alpha2=-pi/2;
arm1.a2=0;
arm1.d2=0;

arm1.Alpha3=pi/2;
arm1.a3=0;
arm1.d3=400; %2


arm1.Alpha4=pi/2;
arm1.a4=0;
arm1.d4=0;

arm1.Alpha5=-pi/2;
arm1.a5=-100; %3 abs
arm1.d5=350; %4

arm1.Alpha6=pi/2;
arm1.a6=0;
arm1.d6=0;

arm1.Alpha7=pi/2;
arm1.a7=100; %5
arm1.d7=0;

arm1.H7=50;%length of joint7

arm1.d8=-40; %6

%Arm1包络
arm1.Env7_allow=2;
arm1.Env7_allow2=2;

arm1.Env7_w1=150;%圆柱外径
arm1.Env7_w2=150;%圆柱外径
arm1.Env7_w3=120;%圆柱外径
arm1.Env7_w4=120;%圆柱外径
arm1.Env7_w5=100;%圆柱外径
arm1.Env7_w6=100;%圆柱外径
arm1.Env7_w7=90;%圆柱外径


arm1.Env7_H2=75;%T型接头左侧长度
arm1.Env7_H3=75;%T型接头右侧长度


%Tool
Tool.W1=60;Tool.H1=117.5;
Tool.W2=55;Tool.H2=72.68;
Tool.W3=8.4;Tool.H3=460;%Tool.H3=445;
%Continuum

Continuum.L=40;%连续体长度
Continuum.g=15;%手术器械长度


