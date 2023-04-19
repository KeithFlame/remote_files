SL = [100 10 20 15 0.1 5 0.6 0 600]';

[T10, T20]=getT12;
T10(1:3,4)=T10(1:3,4)*1000;
psi=[0 60 pi/4 pi/4 0 0]';
[T,S]=FKcc_2segs_bending_keith(psi,SL);
S2=S;
for i = 1:size(S,1)
    S2(i,:)=(T10*S(i,:)')';
end
figure;hold on;xlabel('x');ylabel('y');zlabel('z');
% PS_2segs_keith(S2,SL,T10*T);
PS_2segs_keith(S,SL,T);