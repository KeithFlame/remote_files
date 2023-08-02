function get_target
SL = [100 10 20 15];
phi=(random('unif',0,1,[50000 1])-0.5)*2*pi;
ls = random('unif',0,1,[50000 1])*50+SL(1);
theta1 = random('unif',0,1,[50000 1])*pi/2;
theta2 = random('unif',0,1,[50000 1])*pi*2/3;
delta1 = (random('unif',0,1,[50000 1])-0.5)*2*pi;
delta2 = (random('unif',0,1,[50000 1])-0.5)*2*pi;
psi = [phi ls theta1 delta1 theta2 delta2];
fpath='./target.log';
fileConfig = fopen(fpath,'a+');
for ipsi = 1:50000
Tend=FKcc_2segs_nobending_keith(psi(ipsi,:),SL);
p=Tend(1:3,4); t=Tend(1:3,3);
vec=[psi(ipsi,:) p' t'];
fprintf(fileConfig,'%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f |||| %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f\n',vec);
end
fclose(fileConfig);