function psi=calcPsiFromQ(q,x)
k1=x(5);k2=x(6);
r1=4;r2=4;
if(q(1) == 0&&q(2) == 0)
    theta1=0;delta1=0;
else
%     cosdelta1=@(delta1) (cos(delta1+pi/3)/cos(delta1-pi/3)-q(2)/q(1));
%     delta1=fsolve(cosdelta1,1);
    m=q(2)/q(1)+1;
    delta1=atan((2-m)/sqrt(3)/m);
    theta1=q(1)/cos(delta1-pi/3)/r1/k1;
end
if(q(3) == 0&&q(4) == 0)
    theta2=0;delta2=0;
else
%     cosdelta2=@(delta2) (cos(delta2+pi/2)/cos(delta2)-q(4)/q(3));
%     delta2=fsolve(cosdelta2,1);
    delta2 = -atan(q(4)/q(3));
    theta2=q(3)/cos(delta2)/r2/k2;
end

if(theta1 <0)
    delta1=delta1+pi;
    theta1=-theta1;
end
if(theta2<0)
    delta2=delta2+pi;
    theta2=-theta2;
end
% if(delta>pi)

psi=[0,0,theta1,delta1,theta2,delta2];
end