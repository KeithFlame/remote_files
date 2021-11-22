function du = getDeltaU(lamda)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.22.2021
% Ver. 1.0
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
Q=getQ(lamda);
h=geth(lamda);

du=pinv(Q)*h;
end

