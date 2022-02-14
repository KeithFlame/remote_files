% l=100e-3;
% theta1=pi/2;
% c=0.5e-3;
% Lr=10e-3;
% L2=20e-3;
% % L1=100e-3;
% k1=0.0191;
% zeta=0.1;
% 

% thetasi=0.001:0.001:6;
% tt=zeros(max(size(thetasi)));
syms thetasi;
syms sumL theta1 L1 Lstem k1;
syms u1 us;
syms c zeta;

ks=k1/zeta;

sTsi=sin(thetasi);
cTsi=cos(thetasi);

d=c*sTsi./(1-cTsi);
lsi=d.*thetasi./sTsi;
lso=l-sumL+d-lsi;
thetaso=(theta1-thetasi).*lso*zeta./(lso*zeta+L1);
theta1o=theta1-thetaso-thetasi;

f=0.5*k1*L1*(theta1o/L1-u1)^2+0.5*ks*(lsi*(thetasi/lsi-us)^2+lso*(thetaso/lso-us)^2+(Lstem-lsi-lso)*us^2);

% f=0.5*k1*(theta1o.^2./L1)+0.5*ks*(thetasi.^2./lsi+thetaso.^2./lso);

figure;
plot(thetasi,f);
xlabel('thetasi'),ylabel('f'),title('\it\theta_s_i vs f');
%%
i=1;
tt=zeros(2000,1);
for thetasi=0.001:0.001:2
    tt(i)=191*thetasi*sin(thetasi) - 191*cos(thetasi) + (191*(thetasi - pi/2 + ((thetasi - pi/2)*(sin(thetasi)/(2000*(cos(thetasi) - 1)) - thetasi/(2000*(cos(thetasi) - 1)) + 3/100))/(10*(thetasi/(20000*(cos(thetasi) - 1)) - sin(thetasi)/(20000*(cos(thetasi) - 1)) + 97/1000)))*((sin(thetasi)/(2000*(cos(thetasi) - 1)) - thetasi/(2000*(cos(thetasi) - 1)) + 3/100)/(10*(thetasi/(20000*(cos(thetasi) - 1)) - sin(thetasi)/(20000*(cos(thetasi) - 1)) + 97/1000)) + ((thetasi - pi/2)*(sin(thetasi)^2/(2000*(cos(thetasi) - 1)^2) - 1/(2000*(cos(thetasi) - 1)) + cos(thetasi)/(2000*(cos(thetasi) - 1)) - (thetasi*sin(thetasi))/(2000*(cos(thetasi) - 1)^2)))/(10*(thetasi/(20000*(cos(thetasi) - 1)) - sin(thetasi)/(20000*(cos(thetasi) - 1)) + 97/1000)) + ((thetasi - pi/2)*(sin(thetasi)/(2000*(cos(thetasi) - 1)) - thetasi/(2000*(cos(thetasi) - 1)) + 3/100)*(sin(thetasi)^2/(20000*(cos(thetasi) - 1)^2) - 1/(20000*(cos(thetasi) - 1)) + cos(thetasi)/(20000*(cos(thetasi) - 1)) - (thetasi*sin(thetasi))/(20000*(cos(thetasi) - 1)^2)))/(10*(thetasi/(20000*cos(thetasi) - 20000) - sin(thetasi)/(20000*cos(thetasi) - 20000) + 97/1000)^2) + 1))/1000 - (191*(2*thetasi - pi)*(sin(thetasi)/(2000*(cos(thetasi) - 1)) - thetasi/(2000*(cos(thetasi) - 1)) + 3/100))/(200000*(thetasi/(20000*cos(thetasi) - 20000) - sin(thetasi)/(20000*cos(thetasi) - 20000) + 97/1000)^2) - (191*(thetasi - pi/2)^2*(sin(thetasi)^2/(2000*(cos(thetasi) - 1)^2) - 1/(2000*(cos(thetasi) - 1)) + cos(thetasi)/(2000*(cos(thetasi) - 1)) - (thetasi*sin(thetasi))/(2000*(cos(thetasi) - 1)^2)))/(200000*(thetasi/(20000*cos(thetasi) - 20000) - sin(thetasi)/(20000*cos(thetasi) - 20000) + 97/1000)^2) - (191*(thetasi - pi/2)^2*(sin(thetasi)/(2000*(cos(thetasi) - 1)) - thetasi/(2000*(cos(thetasi) - 1)) + 3/100)*(sin(thetasi)^2/(20000*(cos(thetasi) - 1)^2) - 1/(20000*(cos(thetasi) - 1)) + cos(thetasi)/(20000*(cos(thetasi) - 1)) - (thetasi*sin(thetasi))/(20000*(cos(thetasi) - 1)^2)))/(100000*(thetasi/(20000*cos(thetasi) - 20000) - sin(thetasi)/(20000*cos(thetasi) - 20000) + 97/1000)^3) + 191;
    i=i+1;
end
