function [psi]=limitPsiNum(psi)

if(psi(3)<0)
    psi(3)=-psi(3);
    psi(4)=psi(4)+pi;
end
if(psi(5)<0)
    psi(5)=-psi(5);
    psi(6)=psi(6)+pi;
end

while(psi(4)>2*pi)
    psi(4)=psi(4)-2*pi;
end
while(psi(4)<-2*pi)
    psi(4)=psi(4)+2*pi;
end
while(psi(6)>2*pi)
    psi(6)=psi(6)-2*pi;
end
while(psi(6)<-2*pi)
    psi(6)=psi(6)+2*pi;
end
    
end