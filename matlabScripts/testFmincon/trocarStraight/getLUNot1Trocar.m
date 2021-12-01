function [L,u]=getLUNot1Trocar(theta,l)
if(theta==0||l==0)
    L=l;
    u=[0 0 0]';
else
    L=l;
    u=[0,theta/l,0]';
end