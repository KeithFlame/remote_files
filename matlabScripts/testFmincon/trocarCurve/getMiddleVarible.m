function L_res=getMiddleVarible(l1i,lsi)

persistent L;
if(nargin==0)
    L_res=L;
    return;
end
L=[l1i,lsi];
L_res=L;
end