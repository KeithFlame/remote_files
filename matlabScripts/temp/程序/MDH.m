function [T]=MDH(alpha,a,xita,d)
%MDH�ı任����
T=[cos(xita) -sin(xita) 0 a;
   sin(xita)*cos(alpha) cos(xita)*cos(alpha) -sin(alpha) -d*sin(alpha);
   sin(xita)*sin(alpha) cos(xita)*sin(alpha) cos(alpha) d*cos(alpha);
   0 0 0 1];
end