function [u_fit]=fitCurvature(u,FBG,oneSegIdx)
if(nargin == 2 || oneSegIdx == 0)
u_fit=zeros(3,66);
u_fit(3,1:39)=0:1e-3:0.038;
u_fit(3,39)=0.0375;
u_fit(3,40:65)=0.04:1e-3:0.065;
u_fit(3,66)=0.07;

u_fit(1,1:39)=spline(FBG.s(1:4),[0 u(1,1:4) 0],u_fit(3,1:39));
u_fit(1,40:65)=spline(FBG.s(5:7),[0 u(1,5:7) 0],u_fit(3,40:65));
%u_fit(1,1:65)=spline(FBG.s(1:7),[0 u(1,1:7) 0],u_fit(3,1:65));
u_fit(1,66)=u_fit(1,65);

u_fit(2,1:39)=spline(FBG.s(1:4),[0 u(2,1:4) 0],u_fit(3,1:39));
u_fit(2,40:65)=spline(FBG.s(5:7),[0 u(2,5:7) 0],u_fit(3,40:65));
%u_fit(2,1:65)=spline(FBG.s(1:7),[0 u(2,1:7) 0],u_fit(3,1:65));
u_fit(2,66)=u_fit(2,65);

else
    u_fit=zeros(3,102);
    u_fit(3,1:101)=0:1e-3:0.1;
    u_fit(3,102)=0.12;
    u_fit(1,1:101)=spline(FBG.s(1:10),[0 u(1,1:10) 0],u_fit(3,1:101));
    u_fit(2,1:101)=spline(FBG.s(1:10),[0 u(2,1:10) 0],u_fit(3,1:101));
    u_fit(1,102)=u_fit(1,101);
end

figure(2);hold on;
plot(FBG.s(:),u(1,:),'or');
plot(u_fit(3,:),u_fit(1,:),'-r')
end