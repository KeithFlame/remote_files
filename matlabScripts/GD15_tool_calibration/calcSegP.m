function [p]=calcSegP(theta,delta,L)
if(abs(theta)<1e-7)
    p=[0 0 L]';
else
    p=L/theta*[cos(delta)*(1-cos(theta)) sin(delta)*(1-cos(theta)) sin(theta)]';
end
end