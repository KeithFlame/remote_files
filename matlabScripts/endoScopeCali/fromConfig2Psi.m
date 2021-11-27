function endoPsi=fromConfig2Psi(T,x,gamma3)

% L0,L1,Lr,L2,Lg,gamma1,gamma2,K1,K2
% x0=[5 18 8 30 20 -pi/4 -pi/2 1 1];
L1=x(1);
Lr=x(2);
L2=x(3);
Lg=x(4);
psi=zeros(1,6);
endoPsi=zeros(size(T,3),6);
gamma3T=[cos(gamma3), -sin(gamma3),0,0;sin(gamma3), cos(gamma3),0,0;0, 0, 1, 0;0, 0, 0, 1];
for i =1:size(T,3)
    Tc= T(:,:,i);
    Tcu=Tc*gamma3T;
    para=invKine(Tcu,L1,Lr,L2,Lg);
    
    psi(1)=para(1);
    psi(2)=para(2);
    psi(3)=para(3);
    psi(4)=para(5);
    psi(5)=para(7);
    psi(6)=para(9);
    endoPsi(i,:)=psi;
end


