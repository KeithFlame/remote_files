function psi = scalePsi(psi,psi_limit)
%SCALEPSI 此处显示有关此函数的摘要
%   此处显示详细说明
    psi([1 3 4 5 6 ]) = psi([1 3 4 5 6 ]) / 180*pi;
    tp = abs(psi) - psi_limit;
    v = find(tp>0);
    psi(v) = psi_limit(v).*sign(psi(v));
end

