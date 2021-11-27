function [ S ] = S( vec )
%UNTITLED Summary of this function goes here
%   get the skew symmetric matirx of the input vector
vector = vec;
S=[0 -vector(3,1) vector(2,1);
   vector(3,1) 0 -vector(1,1);
   -vector(2,1) vector(1,1) 0;];

end

