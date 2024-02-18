function [Tb_0_1,Tb_1_2,Tb_2_3,Tb_3_4,Tb_4_5,Tb_5_6,Tb_6_7,Tb_7_8,Tbase1,Tbase2,Tbase3,Tbase4,Tbase5,Tbase6,Tbase7,Tbase8]=FK_7DOF_Offset(arm,Tbase,a)
a1=a(1);
a2=a(2);
a3=a(3);
a4=a(4);
a5=a(5);
a6=a(6);
a7=a(7);

Tb_0_1=MDH(arm.Alpha1,arm.a1,a1,arm.d1);
Tbase1=Tbase*Tb_0_1;
Tb_1_2=MDH(arm.Alpha2,arm.a2,a2,arm.d2);
Tbase2=Tbase1*Tb_1_2;
Tb_2_3=MDH(arm.Alpha3,arm.a3,a3,arm.d3);
Tbase3=Tbase2*Tb_2_3;

Tb_3_4=MDH(arm.Alpha4,arm.a4,a4,arm.d4);
Tbase4=Tbase3*Tb_3_4;

Tb_4_5=MDH(arm.Alpha5,arm.a5,a5,arm.d5);
Tbase5=Tbase4*Tb_4_5;

Tb_5_6=MDH(arm.Alpha6,arm.a6,a6,arm.d6);
Tbase6=Tbase5*Tb_5_6;

Tb_6_7=MDH(arm.Alpha7,arm.a7,a7,arm.d7);
Tbase7=Tbase6*Tb_6_7;


Tb_7_8=MDH(0,0,0,arm.d8);
Tbase8=Tbase7*Tb_7_8;

end