function [L,u]=getLU1Trocar(theta,d)
if(theta==0||d==0)
    L=d;
    u=[0 0 0]';
else
    L=d;
    u=[0,theta/L,0]';
end
end
