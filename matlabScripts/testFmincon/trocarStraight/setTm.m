function T=setTm(x)

persistent T_m;
if(nargin==0)
    T=T_m;
    return;
end
T_m=x;
end
