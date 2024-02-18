function [R] = EulerToRotMat(euler)
r=euler(1);
p=euler(2);
y=euler(3);
R=Expm([0 0 1]'*y/180*pi)*Expm([0 1 0]'*p/180*pi)*Expm([1 0 0]'*r/180*pi);

end