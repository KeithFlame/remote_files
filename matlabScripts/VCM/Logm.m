function [u]=Logm(R)
%simplified calculation for logarithmic map (u in R3)
theta = acos((R(1,1)+R(2,2)+R(3,3)-1)/2);
t=real(theta);
if t==0
    u=[0 0 0]';
else
    u=[R(3,2)-R(2,3) R(1,3)-R(3,1) R(2,1)-R(1,2)]'/2/sin(t)*t;
end

end