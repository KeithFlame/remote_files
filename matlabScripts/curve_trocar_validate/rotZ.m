function T=rotZ(x)

x=x/180*pi;
T=[cos(x) -sin(x) 0 0;sin(x) cos(x) 0 0;0 0 1 0; 0 0 0 1];
end