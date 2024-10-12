% alpha = 1:0.1:179;

beta = [90:0.1:179]';
block_size = length(beta);
L = 100;
step = 2*L/(block_size-1);
d = 0:step:2*L;
d2 = 2 * L;
alpha = 180-atand(d2./d);

A = L./alpha.*tand(alpha);
B = L./beta.*tand(beta);

f = d.*A.*(1-cosd(alpha))+A.*B.*(sind(alpha)-sind(beta))-d.*B.*sind(beta);

fig1 = f>0;

% 
% d=0;
% A = L./alpha.*tand(alpha);
% B = L./beta.*tand(beta);
% 
% f = d*A.*(1-cosd(alpha))+A.*B.*(sind(alpha)-sind(beta))-d.*B.*sind(beta);
% 
% fig2 = f>0;
% 
% fig3 = fig1-fig2;