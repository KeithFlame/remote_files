function u=fromPsi2Curvature(endoPsi,x)



u = zeros(size(endoPsi,1),6);
uc=zeros(1,6);
L1=x(1)*1e-3;
L2=x(3)*1e-3;
for i = 1:size(endoPsi,1)
    if size(endoPsi,2)==4
        temPsi=[0 0 endoPsi(i,:)];
    else
        temPsi=endoPsi(i,:);
    end
    
    uc(1)=temPsi(3)/L1*cos(pi/2-temPsi(4));
    uc(2)=temPsi(3)/L1*sin(pi/2-temPsi(4));
    uc(4)=temPsi(5)/L2*cos(pi/2-temPsi(6));
    uc(5)=temPsi(5)/L2*sin(pi/2-temPsi(6));
    u(i,:)=uc;
end


end