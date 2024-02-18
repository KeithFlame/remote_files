function [JT]=Plot_Joint_T_Offset(arm,W1,W2,H1,H2,H3,Tbase,color)
[hAx]=PlotAxis(100,Tbase);
[HRB1]=PlotRoundBlock(0,W1,H1,Tbase,color);
Trans1=[Expm([pi/2 0 0]') [arm.a4 0 0]';0 0 0 1];
Trans2=[Expm([-pi/2 0 0]') [arm.a4 0 0]';0 0 0 1];
[HRB2]=PlotRoundBlock(0,W2,H2,Tbase*Trans1,color);
[HRB3]=PlotRoundBlock(0,W2,H3,Tbase*Trans2,color);
JT=[HRB1 HRB2 HRB3];
end