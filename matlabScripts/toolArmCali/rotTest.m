function [R] = rotTest(x,y,z)
    R1=[1 0 0;0 cos(x) -sin(x);0 sin(x) cos(x)];
    R2 = [cos(y) 0 sin(y);0 1 0;-sin(y) 0 cos(y)];
    R3 = [cos(z) -sin(z) 0; sin(z) cos(z) 0; 0 0 1];
    R = R3*R2*R1;
end