function [vec7] = fromT2vec7(T)
%FROMT2VEC7 此处显示有关此函数的摘要
%   此处显示详细说明
r=T(1:3,1:3);
p=T(1:3,4)*1e3;
quat=rotm2quat(r);
vec7=[p' quat];

end

