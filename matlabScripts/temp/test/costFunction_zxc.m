function f = costFunction_zxc(x)
data = load('camera consistency X_0307 edited.log');
position = data(:,1:3);
block_size = size(position,1);
true_data=[
 0 0 0
 5 0 0
10 0 0
15 0 0
20 0 0
25 0 0
30 0 0
35 0 0
40 0 0
45 0 0
50 0 0
55 0 0
60 0 0
65 0 0
70 0 0
75 0 0
80 0 0
85 0 0
90 0 0
95 0 0
100 0 0
105 0 0
110 0 0
];
R = eul2rotm(x(1:3)');
p =x(4:6)'*10;
err_position = zeros(block_size,3);
err_p = zeros(block_size,1);
for i = 1:block_size
    err_position(i,:) = position(i,:)-(R*true_data(i,:)'+p')';
    err_p(i) = norm(err_position(i,:));
end
f = mean(err_p);
% f = max(err_p);
end