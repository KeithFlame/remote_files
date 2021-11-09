function endoPsi=fromCurvature2Psi(u,x,temPsi)

% L0,L1,Lr,L2,Lg,gamma1,gamma2,K1,K2
% x0=[5 18 8 30 20 -pi/4 -pi/2 1 1];
L1=x(1)*1e-3;
L2=x(3)*1e-3;
zeta=x(5);
psi=zeros(1,6);
endoPsi=zeros(size(u,1),6);

for i = 1:size(u,1)
    u_=u(i,:);
    if(temPsi(2)<x(1))
        L1=temPsi(2)*1e-3+1e-15;
        psi(3)=L1*norm(u_(1:3));
    else
        psi(3)=(L1+zeta*(temPsi(2)*1e-3-L1))*norm(u_(1:3));
    end
    psi(4)=-pi/2+atan2(u_(2),u_(1));
    psi(5)=L2*norm(u_(4:6));
    psi(6)=-pi/2+atan2(u_(5),u_(4));
    endoPsi(i,:)=psi;
end
end

