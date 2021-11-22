function Q=getQ(lamda)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.22.2021
% Ver. 1.0
% input1: lagrange multipliers
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

[m,n]=size(lamda);
block_size=max([m n]);
lamda=reshape(lamda,[block_size 1]);
Xk=getXk(Pi,Pi_1);
K=getK;

Q=K;
for i =1:block_size
    Q=Q+lamda*(Xk')*Xk;
end

end
