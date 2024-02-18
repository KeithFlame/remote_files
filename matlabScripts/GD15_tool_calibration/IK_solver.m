function [psi,flag]=IK_solver(Tg,x)
%IK solver for ACC test only
psi_cur = [100 0 0 0 0 0]';
seg=[x(3:5);15];
ga=x(1);
zeta = 0.15;
d = 8;
[T]= forward(psi_cur,seg,zeta,d,ga);
T_cur = [T.RO_g T.PO_g; 0 0 0 1];
res=[Tg(1:3,4)-T_cur(1:3,4);S_inv(logm(Tg(1:3,1:3)*T_cur(1:3,1:3)'))];
angle_crt = pi/3600;
position_crt = 0.01;
dt = 10e-3;
v_lim = 100;%m/s
w_lim = pi/1.5;%rad/s
v_thr = 1;
w_thr = 2/180*pi;
phi_max=4.71;
phi_min=-4.71;
cnt=0;
flag = 1;
psi_before = psi_cur;
while( (norm(res(4:6))>=angle_crt||norm(res(1:3))>=position_crt))
    cnt=cnt+1;
    %disp(['current Res: ' num2str(norm(res))])
    
    d_x= [res(1:3)/norm(res(1:3))*v_lim;res(4:6)/norm(res(4:6))*w_lim]*dt;
    if(norm(res(1:3))<v_thr)
        d_x(1:3)=res(1:3)*dt;
    end
    if(norm(res(4:6))<w_thr)
        d_x(4:6)=res(4:6)*w_lim/2/w_thr*dt;
    end
     

    [J] = calcJacobian(psi_cur,seg,zeta,d,ga);
    d_psi = RMRC_iter(J,d_x,psi_cur,seg);
    psi_cur = psi_cur + d_psi;
    psi_cur=limitPsiNum(psi_cur);
    if(psi_cur(2)>phi_max)
        psi_cur(2)=phi_max;
    elseif(psi_cur(2)<phi_min)
        psi_cur(2)=phi_min;
    end
    
    [T] = forward(psi_cur,seg,zeta,d,ga);
    T_cur = [T.RO_g T.PO_g; 0 0 0 1];
    res=[Tg(1:3,4)-T_cur(1:3,4);S_inv(logm(Tg(1:3,1:3)*T_cur(1:3,1:3)'))];
    %=== plot ===%
%     pause(0.001);
%     figure(1);cla;
%     PlotAxis(10,eye(4));
%     PlotSnake(psi_cur,seg_lengths,gamma);
%     PlotAxis(6,T_cur);
%     PlotAxis(6,Tg);
%     grid on;
%     axis equal;
%     axis([-30 30 -30 30 -10 90]);
    %============%
    if(cnt>1000)
        flag = 0;
        disp(['target NOT arrived!']);
        break;
    end
end
psi=psi_cur;
end