function T = fromVec72T(vec7)
%FROMVEC72T 此处显示有关此函数的摘要
%   此处显示详细说明
p = vec7(1:3)';
quat=vec7(4:7);
T=eye(4);
T(1:3,1:3)=quat2rotm(quat);
T(1:3,4)=p;

end

