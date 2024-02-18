function [distance]=TwoLineDistance(a,b,c,d)

% the distance between ab and cd

AB=b-a;
CD=d-c;

V_AB=AB/norm(AB);
V_CD=CD/norm(CD);

N=cross(V_AB,V_CD);

AC=c-a;

distance=abs(dot(N,AC)/norm(N));

% plot3([c(1) d(1)],[c(2) d(2)],[c(3) d(3)],'--o');
% hold on
% plot3([a(1) b(1)],[a(2) b(2)],[a(3) b(3)],'--o');
end