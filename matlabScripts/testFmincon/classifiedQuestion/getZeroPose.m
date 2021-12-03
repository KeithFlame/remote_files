function r=getZeroPose(theta1)
SP=setInitValV2;
us_b=SP.dependent_psi.us_b;
zeta=SP.structure.zeta;
L_line2=SP.trocar.line_2_length;
L1=SP.structure.L1;
rs_b=1/us_b(2);
thetas_b=zeta*L_line2/(zeta*L_line2+L1)*theta1;
r=rs_b*(1-cos(thetas_b));
end