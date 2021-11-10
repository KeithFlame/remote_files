function [pos_err,dir_err]=testConfigErr()

load('..\tests\1019\T_m_tr.mat','T_m_tr');
load('..\tests\1102\Psi_actual.mat','Psi_actual');%delta using new.
psi=Psi_actual(1:6,:)';
N=length(Psi_actual);
%seg_len = [100.795 9.5050 19.4 19.8021]'*1e-3;
%seg_len = [97.9382 9.8953 19.4 19.5123]'*1e-3;
seg_len = [102.861 9.0514 19.4 21.984]'*1e-3;
%seg_len=[103.009 9.7714 19.4 24.8526]'*1e-3;
bend_in_trocar = (13.2358)*1e-3;
offset_zero = (-6.5564)*1e-3;
zeta = 0.14642;
%% trocar channel positions
 pc1= [0.0  -7.85  0.0 ]'*1e-3;
 pc2= [7.66 -1.7   0.0 ]'*1e-3;
 pc3= [0.0   5.8 -15.0 ]'*1e-3;
 pc4=[-7.66 -1.7   0.0 ]'*1e-3;
 %% history tau_s
 tau1 = -0.934;
 tau2 = -0.489;
 tau4 = 0.268;
 %% trocar outport w.r.t trocar world
T_ch_tr = [Expm([0 0 tau4+(11/180*pi)]') ...
    pc4;0 0 0 1];%2nd arm of 03 surgical
figure(1);
cla;
hold on;

%% calc and plot
pos_err = zeros(N,1);
ori_err = zeros(N,1);
dp_err = zeros(N-1,1);
do_err = zeros(N-1,1);
qa = zeros(24,N);
i = 1;
for j=1:N
if(T_m_tr(4,4,j)~=0)
    %cla;

    %test0831(1:3,1:3,i) = test0831(1:3,1:3,i)*Expm([0 0 pi]');
    T_m_tr(1:3,4,j)=T_m_tr(1:3,4,j)/1000;
    %delta using old.
    psi_ = [psi(j,2) psi(j,1)/1000 psi(j,3) -psi(j,4) psi(j,5) -psi(j,6)]';
    Tg(1:4,1:4,j) = inv(T_ch_tr)*T_m_tr(1:4,1:4,j);
    %[theta_trocar,len_trocar]=calcThetaInTrocar(psi_,seg_len,zeta,bend_in_trocar,offset_zero);

    qa(:,j) = calcActuation_dual(psi_, 1);
    %--------plot------------%
        PlotAxis(0.01,eye(4));%trocar base considering tau
        PlotAxis(0.01,inv(T_ch_tr));%trocar world frame    
        PlotCircle(5.0e-3,[0 0 0],[0 0 1]);
        PlotAxis(0.005,Tg(1:4,1:4,j));%actual target   
        [Tall]=PlotSnake_trocar(psi_, seg_len, zeta, 0, 0, bend_in_trocar,offset_zero);  
            Tg_(1:4,1:4,j) = Tall.T_tipg;
        PlotAxis(0.005,Tg_(1:4,1:4,j));%theoretical target
        axis equal
        grid on

    pos_err(i) = norm(Tg_(1:3,4,j)-Tg(1:3,4,j))*1000;
    dir_err(i) = norm(acos(Tg_(1:3,3,j)'*Tg(1:3,3,j)))/pi*180;
    i=i+1;
else
end
end
pos_err=pos_err(1:i-1);
dir_err=dir_err(1:i-1);
p_avg = mean(pos_err);
o_avg = mean(dir_err);
disp(['average abs pos err is ' num2str(p_avg) ' mm']);
disp(['average abs ori err is ' num2str(o_avg) ' deg']);

end
