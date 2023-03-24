function q = scaleQ(q,q_limit)
%SCALEPSI 此处显示有关此函数的摘要
%   此处显示详细说明
    tp = abs(q) - q_limit;
    v = find(tp>0);
    q(v) = q_limit(v).*sign(q(v));
end

