
function [g,h]=geth_V3(x)
x=x./[100 100 200];

SP=setInitVal;
c=SP.c;
L1=SP.L1;
Lr=SP.Lr;
L2=SP.L2;
Ls=SP.Ls;
zeta=SP.zeta;
thetasi=x(1);
theta1i=x(2);
d=x(3);
g=-d;

if(Ls>1e-9)
    d1i=0;
    dsi=d;
elseif(d+SP.l-Lr-L2>L1&&SP.l-Lr-L2<L1)
    d1i=L1-(SP.l-Lr-L2);
    dsi=d-d1i;
else
    dsi=0;
    d1i=d;
end
if(dsi==0)
    thetasi=0;
elseif(d1i==0)
    theta1i=0;
else
    %comment it. means ignore zeta.
    tem=sin(theta1i)/d1i*zeta*dsi;
    if(norm(tem)>1||tem<0.001)
%         thetasi=theta1i*zeta;
    else
        thetasi=asin(tem);
    end
    d1i=d-dsi*sin(thetasi)/thetasi;
end

% if(SP.c==0)
%     thetasi=0;
%     theta1i=0;
% end


if(Ls>1e-5)
    g=[g;d/sin(thetasi)*(1-cos(thetasi))-c;-1;-1;-1];
else
    if(thetasi==0)
        d1=0;
    else
        d1=dsi/sin(thetasi)*(1-cos(thetasi));
    end
    if(theta1i==0)
        d2=0;
    else
        d2=2*d1i/cos(thetasi)/sin(theta1i)*sin(theta1i/2)*sin(theta1i/2+thetasi);
    end
    g=[g;d1+d2-c;d1i+(SP.l-SP.L2-SP.Lr-SP.L1)-c;-1;-1];
end

if(SP.Ls<0||dsi<1e-5||(d-dsi)<1e-5)
    h=0;
else
    h=sin(theta1i)*cos(thetasi)*zeta/(d-dsi)-sin(thetasi)/dsi;
end

end