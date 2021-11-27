function Q=calcQFromPsi(endoPsi,x)
k1=x(5);k2=x(6);r1=4;r2=4;
theta1=endoPsi(3);delta1=endoPsi(4);
theta2=endoPsi(5);delta2=endoPsi(6);

q11=r1*k1*theta1*cos(delta1-pi/3);
q12=r1*k1*theta1*cos(delta1+pi/3);
q21=r2*k2*theta2*cos(delta2);
q22=r2*k2*theta2*cos(delta2+pi/2);

Q=[q11 q12 q21 q22];
end


