function [JT]=Plot_Joint_T(W,H1,H2,H3,Tbase,color)
[hAx]=PlotAxis(100,Tbase);
[HRB1]=PlotRoundBlock(0,W,H1,Tbase,color);
Trans1=[Expm([pi/2 0 0]') [0 0 0]';0 0 0 1];
Trans2=[Expm([-pi/2 0 0]') [0 0 0]';0 0 0 1];
[HRB2]=PlotRoundBlock(0,W,H2,Tbase*Trans1,color);
[HRB3]=PlotRoundBlock(0,W,H3,Tbase*Trans2,color);
JT=[HRB1 HRB2 HRB3];
end