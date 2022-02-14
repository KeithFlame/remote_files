function f=costFunc_oritientation(x)

t=load('./data/T_4_trocar.mat');
T_4_trocar=t.T_4_trocar;
block_size=size(T_4_trocar,3);
dis=zeros(block_size,1);
x=reshape(x,[1 3]);
R=eul2rotm(x);
for i =1:block_size
    axang=rotm2axang(R'*T_4_trocar(1:3,1:3,i));
    dis(i)= axang(4)*180/pi;
end
f=max(dis);
end