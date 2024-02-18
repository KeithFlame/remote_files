function T = S( p )
%this function gives the skew-symmetric matrix of the vector p

if(length(p)==3)
    T=[0 -p(3) p(2); p(3) 0 -p(1);-p(2) p(1) 0];
elseif(length(p)==6)
    R=[0 -p(6) p(5); p(6) 0 -p(4);-p(5) p(4) 0];
    T=[R p(1:3);zeros(1,4)];
end
end

