% Ver. 0.2
% added zeta

function u=fromPsi2Curvature(endoPsi,x)



u = zeros(size(endoPsi,1),6);
uc=zeros(1,6);

L1=x(1)*1e-3;
L2=x(3)*1e-3;
zeta=x(5);

for i = 1:size(endoPsi,1)
    
    if size(endoPsi,2)==4
        temPsi=[0 0 endoPsi(i,:)];
    else
        temPsi=endoPsi(i,:);
    end
    if(temPsi(2)<=x(1))
        L1=temPsi(2)*1e-3+1e-15;
        theta1=temPsi(3);
        theta2=temPsi(5);
        delta1=temPsi(4);
        delta2=temPsi(6);
    else
        %theta1=temPsi(3);
        theta1=L1/(zeta*(temPsi(2)*1e-3-L1)+L1)*temPsi(3);
        theta2=temPsi(5);
        delta1=temPsi(4);
        delta2=temPsi(6);
    end    
    
    uc(1)=theta1/L1*cos(pi/2+delta1);
    uc(2)=theta1/L1*sin(pi/2+delta1);
    uc(4)=theta2/L2*cos(pi/2+delta2);
    uc(5)=theta2/L2*sin(pi/2+delta2);
    u(i,:)=uc;
end


end