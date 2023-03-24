function x = tem_T2x(T,noise)
    if (nargin == 1)
        noise = 0;
    end
    p = T(1:3,4);
    r = T(1:3,1:3);
    eul_r = rotm2eul(r);
    p = noise*(rand(3,1)-1) + p;
    eul_r = noise*(rand(1,3)-1)*pi/180 +eul_r;
    x = [p;eul_r'];
end