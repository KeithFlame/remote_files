function f = costFunc_trocar(x)
%COSTFUNC_TROCAR 此处显示有关此函数的摘要
%   此处显示详细说明
p=x(1:3)/1000;
zyx=x(4:6);

R=eul2rotm(zyx);
Ttt_trocar=[R,p';[0 0 0 1]];

[T_m,T_c,~,~]=getMC;
% T_c=Ttt_trocar*T_c;
block_size=max(size(T_m));
dp=zeros(block_size,3);
da=zeros(block_size,5);
for i =1:block_size
    T_c_t=Ttt_trocar*T_c(:,:,i);
    P_tem=(T_m(1:3,4,i)-T_c_t(1:3,4));
    dp(i,:)=reshape(P_tem,[1 3]);
    da(i,4)=norm(dp(i,:))*1000;
    da(i,5)=acosd(dot(T_m(1:3,3,i), T_c_t(1:3,3)));



end
f=(sum(da(:,4))+sum(da(:,5)))/100;
end