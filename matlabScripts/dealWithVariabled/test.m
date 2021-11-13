
l_in=40;
r_L=4;
r_L2=3;
r_trocar=5;
theta_i=0;
thetaii=zeros(1001,1);
ddt=zeros(1001,1);
for i=1:1001
    theta=(i-1)/10000;
    r_curvature_target=l_in/theta;
    dt=2*r_trocar - r_L-r_L2;
    d=l_in;
    r_curvature_actual_limit=r_L2+(d*d+dt*dt)/2/d;
    thetai=atan(dt/d);
    theta_i=thetai;
    if(theta>thetai)
        theta_=thetai;
    else
        theta_=theta;
    end
    
    ddt(i)=r_trocar - r_L2+(r_trocar - r_L)*0.5*(1-cos(theta_/thetai*pi));
    thetaii(i)=atan(ddt(i)/d);
end
plot(thetaii-theta_i)
% plot(ddt)