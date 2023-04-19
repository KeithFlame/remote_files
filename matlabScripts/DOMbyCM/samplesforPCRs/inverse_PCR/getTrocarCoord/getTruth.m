function X_block=getTruth(psi_block)

SL = [100 10 20 15 0.2 5 0.6 0 600]';
num=size(psi_block,2);
X_block = zeros(6,num);
for i = 1:num
    psi=psi_block(:,i);
    T=FKcc_2segs_bending_keith(psi,SL,1);
    x= fromT2X(T);
    X_block(:,i)=x;
end
