function [u]=strain2curvature(delta_comp,Ch1,Ch2,Ch3)
% input planar bending data for compensation
N=18;spacing=10e-3;r=36.5e-6;
%---for manual reading
if(nargin == 1)
Ch1=importdata('./0925/cfg1_150_ch1.txt');
Ch2=importdata('./0925/cfg1_150_ch2.txt');
Ch3=importdata('./0925/cfg1_150_ch3.txt');
end

%------------------------------

num=length(Ch1);
strains=[mean(Ch3,1);mean(Ch1,1);-mean(Ch2,1)];
strains0=mean(strains,1);
strains=strains-[strains0;strains0;strains0];
u=zeros(3,18);

for i=1:18
    %P=[strains(1,i)-0.5*strains(2,i)-0.5*strains(3,i) sqrt(3)/2*(strains(2,i)-strains(3,i)); ...
    %   sqrt(3)/2*(strains(2,i)-strains(3,i)) strains(1,i)+0.5*strains(2,i)+0.5*strains(3,i)];
    proj = pinv([1 0;-1/2 sqrt(3)/2;-1/2 -sqrt(3)/2])*strains(:,i);
    X=proj(1);Y=proj(2);
    delta_p(i)=atan2(Y,X);
    strain_p(i)=sqrt(X^2+Y^2);
    theta_p(i)=strain_p(i)*(1e-6)*spacing/r;
    if(delta_p(i)>pi)
        delta_p(i)=2*pi-delta_p(i);
    end

end
delta_desire=40/180*pi;
delta_p=delta_p-delta_comp;
%delta_p12=delta_p(12);
delta_p12=delta_p(7);
for i=1:18
    delta_p(i)=delta_p(i)-delta_p12+delta_desire;
    %if(i>=16)
    %    delta_p(i)=delta_desire;
    %else
    %    delta_p(i)=delta_desire;
    %end
    u(:,i)=[theta_p(i)/spacing*cos(pi/2+delta_p(i)) theta_p(i)/spacing*sin(pi/2+delta_p(i)) 0]';
end
u=u*1.0;
% figure(1);
% delta_align=-delta_p(12)+pi/2;
% global Ori_1 Posi_1;
% Ori_1=eye(3);Posi_1=zeros(3,1);
% for i=12:18
%     PlotSnake(delta_p(i)+delta_align,theta_p(i),spacing,[5e-4 1e3]);
% end
% axis equal;grid on;
% axis([-60 60 -60 60 0 100]*1e-3);

%u=Expm([0 0 -delta_p(12)]')*u;%in +y
%u=Expm([0 0 -delta_p(12)-pi]')*u;
%u=Expm([0 0 -delta_p(12)-pi-pi/3]')*u;%in -x

end

