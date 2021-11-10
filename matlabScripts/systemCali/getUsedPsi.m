%--------- pose-cmd--------------------%
% to get psi from *.txt
% ----Info
% By Keith W.
% Date 20201110
% Ver c1.0
% input: 不同组实际用到的psi，不是下发的psi。
% output: 不同组的psi文件，按照输入的名字进行保存
%-------------------------------------------------------------------------%
name='Psi_actual_70_4_1';
fpath=['./test1102/',name,'.txt'];
M=load(fpath);
Psi=M';
fname=['./test1102/psi/',name];
save(fname,'Psi');