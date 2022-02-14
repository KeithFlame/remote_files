function f=costFunc_Position(x)

t=load('./data/T_4_trocar.mat');
T_4_trocar=t.T_4_trocar;
block_size=size(T_4_trocar,3);
dis=zeros(block_size,1);
x=reshape(x,[3 1]);
for i =1:block_size
    dis(i)= norm(x-T_4_trocar(1:3,4,i));
end
f=max(dis);
end