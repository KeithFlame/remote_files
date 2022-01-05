function f = costFunc_init_trocar(x)
%COSTFUNC_INIT_TROCAR 此处显示有关此函数的摘要
%   此处显示详细说明
global psi0;

p=x(1:3)/1000;
zyx=x(4:6);

R=eul2rotm(zyx);
Ttt_trocar=[R,p';[0 0 0 1]];

[T_m,~,psi,SP]=getMC;

block_size=max(size(T_m));
dp=zeros(block_size,3);
da=zeros(block_size,2);
for i =1:block_size
    tem_psi=psi(i,:);
    tem_psi_added=tem_psi+psi0;
    T_0=fromPsi2Config(tem_psi_added,SP);
    T=Ttt_trocar*T_0;
    dp(i,:)=reshape(T(1:3,4)-T_m(1:3,4,i),[1 3]);
    da(i,1)=norm(dp(i,:))*1000;
    tem=dot(T(1:3,3),T_m(1:3,3,i));
    if(tem>1)
        tem=1;
    elseif(tem<-1)
        tem=-1;
    end
    da(i,2)=acosd(tem);

end
f=sum(sum(da))/100;
end

