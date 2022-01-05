function f = costFunc_initValue(x)
%COSTFUNC_INITVALUE 此处显示有关此函数的摘要
%   此处显示详细说明

global Ttt_trocar;
x(2)=x(2)/1000;
x(3)=x(3);x(5)=x(5);
[T_m,~,psi,SP]=getMC;

block_size=max(size(T_m));
dp=zeros(block_size,3);
da=zeros(block_size,2);
for i =1:block_size
    tem_psi=psi(i,:);
    tem_psi_added=tem_psi+x';
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
f= sum(sum(da(:,1)))/100;
% f= max(da(:,1))/100;
end

