x=zeros(6,1);
x=[10.8552    6.0153   11.1089 -131.1270   93.7750   68.8071]';
[x1,y1,ef1]=fmincon(@get2coords,x);

T=fromX2T(x1),
function f=get2coords(x)

Pec1=[1 2 3
    4 5 6 
    7 4 9 ];
Pec2=[ 7 7 9
    4 5 10 
    1 2 -4 ];
block_size = size(Pec1,1);
% Pec11=[Pec1 ones(block_size,1)];
% Pec21=[Pec2 ones(block_size,1)];
Tec12=fromX2T(x);
Rec12=Tec12(1:3,1:3);
Pec12=Tec12(1:3,4);
pos_err=zeros(block_size,1);
pos_err_vec=zeros(block_size,3);
for i = 1 :block_size
    posi = Pec1(i,:)'-(Rec12*Pec2(i,:)'+Pec12);
    pos_err_vec(i,:)=posi;
    pos_err(i)=norm(posi);
end
f=sum(pos_err);
end