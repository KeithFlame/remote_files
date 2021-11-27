function a_mat = str_rep(a)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明

num_line = size(a, 1);
a_mat = zeros(num_line, 27);
for i= 1: num_line
    a_i = a{i,1};
    a_i = replace(a_i, '[', '');
    a_i = replace(a_i, ']', ',');
    a_i = replace(a_i, '\t', '');
    a_mat(i, :) = str2num(a_i);
end
end

