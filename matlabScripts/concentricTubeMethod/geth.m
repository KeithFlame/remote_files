function h=geth(lamda,Pi,Pi_1)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.22.2021
% Ver. 1.0
% input1: a point position attched on outer tube 
% input1: a point position attched on inner tube
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
[m,n]=size(lamda);
block_size=max([m n]);
lamda=reshape(lamda,[block_size 1]);
Xk=getXk(Pi,Pi_1);
xkT=getxkT(Pi,Pi_1);
g=getg;


h=K;
for i =1:block_size
    h=h+lamda*(Xk')*(xkT');
end