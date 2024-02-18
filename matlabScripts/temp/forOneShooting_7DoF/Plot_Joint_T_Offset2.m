function [JT]=Plot_Joint_T_Offset2(arm,W1,W2,H1,H2,H3,Tbase,color)
[hAx]=PlotAxis(100,Tbase);
Trans1=[Expm([pi/2 0 0]') [arm.a5 0 0]';0 0 0 1];
Trans2=[Expm([0 0 0]') [0 0 0]';0 0 0 1];
Trans3=[Expm([pi 0 0]') [0 0 0]';0 0 0 1];
[HRB1]=PlotRoundBlock(0,W1,H1,Tbase*Trans1,color);
[HRB2]=PlotRoundBlock(0,W2,H2,Tbase*Trans2,color);
[HRB3]=PlotRoundBlock(0,W2,H3,Tbase*Trans3,color);
JT=[HRB1 HRB2 HRB3];
end