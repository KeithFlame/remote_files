clc;clear;cla;
%======test for JacobianCoupled with resolved rates
psi=[0 3e-3 0 0 0 0]';
pg=[(rand(2,1)-[0.5 0.5]')/15; rand*0.01+0.1];Rg=expm(S(2*rand(3,1)-1*ones(3,1)));
Tg=[Rg pg;0 0 0 1];
[R,p]=taskForward(psi);
T=[R p;0 0 0 1];
res=[pg-p;invS(logm(Rg*R'))];
vlim=200e-3;
wlim=pi/2;
dt=1e-2;
vthr=2e-3;
wthr=5/180*pi;
while(norm(res(1:3))>5e-4 || norm(res(4:6))>0.5/180*pi)
    disp(['current Res: ' num2str(norm(res))])
    x_dot = [res(1:3)/norm(res(1:3))*vlim;res(4:6)/norm(res(4:6))*wlim];
    if(norm(res(1:3))<vthr)
        x_dot(1:3)=res(1:3);
    end
    if(norm(res(4:6))<wthr)
        x_dot(4:6)=res(4:6)*wlim/2/wthr;
    end
    J=getJacobianCoupled(psi);
    %Jacobian converts to mm unit
    J1=J;
    J1(1:3,1)=J(1:3,1)*1e3;
    J1(4:6,2)=J(4:6,2)*1e-3;
    J1(1:3,3:6)=J(1:3,3:6)*1e3;
    
    if det(J1)<=1e-4
        invJ=J'*inv(J*J'+2e-3*eye(6));
    else
        invJ=inv(J);
    end
    psi_dot=invJ*x_dot;
    psi=psi+psi_dot*dt;
    
    
    [R,p]=taskForward(psi,Tg);
    res=[pg-p;invS(logm(Rg*R'))];
    pause(0.005);
    
end