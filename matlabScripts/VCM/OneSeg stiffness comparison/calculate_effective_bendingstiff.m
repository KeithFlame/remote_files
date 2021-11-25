clc;clear;
for j=1:21
for i=2:21
    %-===bending stiffness
    L=0.02+(j-1)*2e-3;
    theta_t=(i-1)*pi/40;
    q=[0 0 0 0]';
    [theta_,p0]=BottomUpCosserat_OneSeg_for(q, L,[0 0 0]');
    while(theta_-theta_t<-1e-3)
        dq = (theta_t-theta_)*0.85e-3;
        q=q+[-dq 0 0 0]';
        [theta_,p0]=BottomUpCosserat_OneSeg_for(q, L,[0 0 0]');
        
    end

    [theta_,p1]=BottomUpCosserat_OneSeg_for(q, L,[0 0 0.05]');

    stiff(i-1,j)=0.05/norm(p1-p0);
    u=theta_t/L;
    K=4*50e9*pi*(0.4e-3)^4/64;%4EI
    K2=50e9*( 4*pi*(0.4e-3)^4/64+2*pi*(0.4e-3)^2/4*(0.85e-3)^2 );%4EI+2*Ar^2
    [u0_sr,p0_sr]=SingleRod(L,[0 K*u 0]',[0 0 0]');
    [u1_sr,p1_sr]=SingleRod(L,[0 K*u 0]',[0 0 0.05]');
    stiff_sr(i-1,j)=0.05/norm(p1_sr-p0_sr);
    
    [u0_sr,p0_sr]=SingleRod(L,[0 K2*u 0]',[0 0 0]',K2);
    [u1_sr,p1_sr]=SingleRod(L,[0 K2*u 0]',[0 0 0.1]',K2);
    stiff_sr2(i-1,j)=0.1/norm(p1_sr-p0_sr);
   
end
end
%-===for euclidean stiffness
L=linspace(0.02,0.06,21);
q=linspace(4.5,90,20);
[X,Y]=meshgrid(L,q);
figure(1);hold on;
surf(X,Y,stiff_sr);
surf(X,Y,stiff);
surf(X,Y,stiff_sr2);
grid on;
xlabel('Segment length (m)');
ylabel('Bending angle (deg)')
zlabel('Stiffness (N/m)')
%-===for bending stiffness