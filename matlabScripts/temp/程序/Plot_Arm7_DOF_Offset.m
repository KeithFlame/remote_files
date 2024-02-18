function [Arm_7DOF_Offset,Tbase7]=Plot_Arm7_DOF_Offset(arm,Tool,Tbase,a)
[Tb_0_1,Tb_1_2,Tb_2_3,Tb_3_4,Tb_4_5,Tb_5_6,Tb_6_7,Tb_7_8,Tbase1,Tbase2,Tbase3,Tbase4,Tbase5,Tbase6,Tbase7,Tbase8]=FK_7DOF_Offset(arm,Tbase,a);
axis equal
color=[1 0.5 0];
color2=[0 1 1];
[JT1]=Plot_Joint_T(arm.W1,arm.d1,arm.H2_1,arm.H2_2,Tbase1,color);
[JT2]=Plot_Joint_T2(arm.W3,arm.W2,arm.d3/2,arm.H2_1,arm.H2_2,Tbase2,color);
[JT3]=Plot_Joint_T_Offset(arm,arm.W3,arm.W4,arm.d3/2,arm.H4_1,arm.H4_2,Tbase3,color);
[JT4]=Plot_Joint_T_Offset2(arm,arm.W5,arm.W4,arm.d5/2,arm.H4_1,arm.H4_2,Tbase4,color);
[JT5]=Plot_Joint_T(arm.W6,arm.d5/2,arm.H6_1,arm.H6_2,Tbase5,color);
[JT6]=Plot_Joint_T_Offset3(arm,arm.W7,arm.W6,arm.H7,arm.H6_1,arm.H6_2,Tbase6,color);
[JT7]=PlotRoundBlock(0,60,-arm.d8,Tbase7,color2);
WT1=Tool.W1;HT1=Tool.H1;
WT2=Tool.W1;HT2=Tool.H2;
WT3=Tool.W3;HT3=Tool.H3;
[Instr]=Plot_Instr(WT1,HT1,WT2,HT2,WT3,HT3,Tbase8);
Arm_7DOF_Offset=[JT1 JT2 JT3 JT4 JT5 JT6 JT7 Instr];
end