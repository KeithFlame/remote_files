function dZt = scaleDZT(dZt,tar_limit)
%     dv = dZt(1:3)/norm(dZt(1:3));
%     dw = dZt(4:6)/norm(dZt(4:6));
    d = dZt/norm(dZt);
    dZt = d.*tar_limit;
end