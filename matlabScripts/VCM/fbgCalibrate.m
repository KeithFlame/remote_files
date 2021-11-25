function [delta_comp]=fbgCalibrate()
% input planar bending data for compensation
N=18;spacing=10e-3;r=36.5e-6;
Ch1=importdata('./0925/planar_ch1.txt');
Ch2=importdata('./0925/planar_ch2.txt');
Ch3=importdata('./0925/planar_ch3.txt');
num=length(Ch1);
strains=[mean(Ch3,1);-mean(Ch2,1);mean(Ch1,1)];
strains0=mean(strains,1);
strains=strains-[strains0;strains0;strains0];
for i=1:18
    proj = pinv([1 0;-1/2 sqrt(3)/2;-1/2 -sqrt(3)/2])*strains(:,i);
    X=proj(1);Y=proj(2);
    delta_p(i)=atan2(Y,X);
    strain_p(i)=sqrt(X^2+Y^2);
    theta_p(i)=strain_p(i)*1e-6*spacing/r;
    if(delta_p(i)>pi)
        delta_p(i)=2*pi-delta_p(i);
    end
end
delta_comp=delta_p-delta_p(12);%for compensation
end

