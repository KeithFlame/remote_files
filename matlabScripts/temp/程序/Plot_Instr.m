function [Instr]=Plot_Instr(W1,H1,W2,H2,W3,H3,Tbase)

%[hAx]=PlotAxis(30,Tbase);
Trans1=[Expm([pi 0 0]') [0 0 0]';0 0 0 1];
Tbase1=Tbase*Trans1;
[HRB1]=PlotRoundBlock(0,W1,H1,Tbase1,[0.6667 0.6667 1]);
Trans2=[Expm([pi 0 0]') [0 0 H1]';0 0 0 1];
[HRB2]=PlotRoundBlock(0,W2,H2,Tbase*Trans2,[1 0.6667 1]);
Trans3=[Expm([pi 0 0]') [0 0 H1+H2]';0 0 0 1];
[HRB3]=PlotRoundBlock(0,W3,H3,Tbase*Trans3,[0 0 0]);
Instr=[HRB1 HRB2 HRB3];