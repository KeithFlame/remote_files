
psi = [phi lf theta1 delta1 theta2 delta2];
rL_para=getToolArmStructureParameter;

if(lf<0)
    Ttip=eye(4);
    return;
end
if(lf<rL_para(8))
    Ttip=eye(4);
    Ttip(3,4)=lf;
    return;
end
if(lf<sum(rL_para([6 8])))
    



theta1=2 /180*pi;
L1=100;




t=2;
r=4;
ld=0:100;
block_size=size(ld,2);
Lt=zeros(block_size,1);
for i =1:block_size
    theta1_in = ld(i)/L1*theta1;
    alpha=atan(t/ld(i));
    R=ld(i)/sin(alpha);
    if(alpha<theta1_in)
        Lt(i)=(R+r)*alpha;
    else
        Lt(i)=0;
    end
end
    
figure;plot(ld,Lt);
