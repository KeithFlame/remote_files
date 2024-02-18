function [invJ]=svd_inv(J)
lambda = 1e-4;
crt = 1e-3;

n = min(size(J));

[u,sigma,v]=svd(J);

for i=1:n
    if(abs(sigma(i,i))<=crt)
        sigma(i,i)=1/(sigma(i,i)+lambda);
    else
        sigma(i,i)=1/sigma(i,i);
    end
end
invJ=(v*sigma'*u');
end