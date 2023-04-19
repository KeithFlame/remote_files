function x = getCur



x_quat=[
    
]';
num = size(x_quat,2);
x = zeros(6,num);
for i = 1:num
    vec= x_quat(:,i);
    Ti=fromQuat2T_mm(vec);
    x(:,i)=fromT2X(Ti);
end

end
