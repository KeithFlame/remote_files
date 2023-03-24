function [ x, r, k ] = CorGrant( A,x0,  b )
    x = x0;
    r = b - A * x0;
    d = r;
    X = ones(length(x), 1);
    k = 0;
    while 1
        if norm(r, Inf)<1e-6
            break
        end
        k = k + 1;
        
        arf = (r' * r) / (d' * A * d);
        x = x + arf * d;
        r2 = r - arf * A * d;
        brt = (r2' * r2) / (r' * r);
        d = r2 + brt * d;
        
        r = r2;
    end

end
