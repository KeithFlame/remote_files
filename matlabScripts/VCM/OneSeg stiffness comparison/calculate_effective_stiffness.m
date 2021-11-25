clc;clear;
for j=1:21
for i=1:21
    %-===bending stiffness
%     L=0.04+j*10e-3;
%     u0=BottomUpCosserat_OneSeg_for([0 0 -(i-1)*0.21e-3 0]', L,[0 0 0]');
%     u0_avg=mean(u0(2,:));
%     u1=BottomUpCosserat_OneSeg_for([0 0 -(i-1)*0.21e-3 0]', L,[0 0.01 0]');
%     u1_avg=mean(u1(2,:));
%     stiff(i,j)=0.01/(u1_avg-u0_avg);
% 
%     u0_sr=SingleRod(L,[0 7.996396e-3/L*pi/40*(i-1) 0]');
%     u0_sr_avg=mean(u0_sr(2,:));
%     u1_sr=SingleRod(L,[0 7.996396e-3/L*pi/40*(i-1)+0.01 0]');
%     u1_sr_avg=mean(u1_sr(2,:));
%     stiff_sr(i,j)=0.01/(u1_sr_avg-u0_sr_avg);
    
    %-====euclidean stiffness: x-direction
    L=0+(j-1)*2e-3;
    [u0,p0]=BottomUpCosserat_OneSeg_for([0 0 -(i-1)*6.8094e-5 0]', L,[0 0 0]',[0 0 0]');
    [u1,p1]=BottomUpCosserat_OneSeg_for([0 0 -(i-1)*6.8094e-5 0]', L,[0 0 0]',[0.01 0 0]');
    stiff(i,j)=0.01/norm(p1-p0);
    
%     [u0_sr,p0_sr]=SingleRod(L,[0 0.4872/L*pi/40*(i-1) 0]',[0 0 0]');
%     [u1_sr,p1_sr]=SingleRod(L,[0 0.4872/L*pi/40*(i-1) 0]',[0 0 0.01]');
    [u0_sr,p0_sr]=SingleRod(L,[0 1.84e-4/0.05*pi/40*(i-1) 0]',[0 0 0]');
    [u1_sr,p1_sr]=SingleRod(L,[0 1.84e-4/0.05*pi/40*(i-1) 0]',[0.01 0 0]');
    
    stiff_sr(i,j)=0.01/norm(p1_sr-p0_sr);
end
end
%-===for euclidean stiffness
L=linspace(0.0,0.04,21);
q=linspace(0,90,21);
[X,Y]=meshgrid(L,q);
figure(3);hold on;
surf(X,Y,stiff);
surf(X,Y,stiff_sr);
grid on;
xlabel('Segment length (m)');
ylabel('Bending angle (deg)')
zlabel('Stiffness (N/m)')
%-===for bending stiffness
% L=linspace(0.05,0.24,20);
% i=1;figure(7);hold on
% plot(L,stiff(1,:),'-sb');
% plot(L,stiff_sr(1,:),'-sy');