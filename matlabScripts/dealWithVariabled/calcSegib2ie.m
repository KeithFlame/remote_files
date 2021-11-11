function calcSegib2ie(rLi, Li, thetai,deltai,r_trocar)

if(nargin==4)
    r_trocar=10*1/2;
end

ld=Li;
t=2*(r_trocar-rLi);
r=rLi;