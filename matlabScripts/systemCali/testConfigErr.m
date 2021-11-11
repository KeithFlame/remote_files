function [pos_err,dir_err]=testConfigErr(name,options)
% name='33_4_2';
if (nargin==1)
    options=1;
end
split_name = regexp(name, '_', 'split');
pose_path=['./test1102/pose/Ttrocar_marker_data_',name,'.mat'];
psi_path=['./test1102/psi/Psi_actual_',name,'.mat'];
t=load(pose_path);
T_m_tr=t.Ttrocar_marker;
t=load(psi_path);
Psi_actual=t.Psi;

optimal_path='./test1102/optimal_res.mat';
t=load(optimal_path);
optimal_res=t.optimal_res;
[~,po]=min(abs(optimal_res(:,1)-str2double(char(split_name(1)))));
structure_para2=optimal_res(po,2:12);
structure_para=structure_para2;
if(options==0)
    optimal_path=['./test1102/optimal_res_',name,'.mat'];
    t=load(optimal_path);
    optimal_res=t.optimal_res;
    structure_para=optimal_res(1:11);
elseif(options==2)
    optimal_path=['./test1102/optimal_res_nogamma1_',name,'.mat'];
    t=load(optimal_path);
    optimal_res=t.optimal_res;
    structure_para=optimal_res(1:11);
end

psi=Psi_actual(1:6,:)';
N=length(Psi_actual);

seg_len = structure_para(1:4)'*1e-3;
bend_in_trocar = -structure_para(11)*1e-3;
offset_zero = structure_para(10)*1e-3;

% if(options==0)
%     bend_in_trocar = -structure_para(11)*1e-3;
%     offset_zero = structure_para(10)*1e-3;
% end




zeta = structure_para(5);
%% trocar channel positions
 pc1= [0.0  -7.85  0.0 ]'*1e-3;
 pc2= [7.66 -1.7   0.0 ]'*1e-3;
 pc3= [0.0   5.8 -15.0 ]'*1e-3;
 pc4=[-7.66 -1.7   0.0 ]'*1e-3;
 Pc=[pc1 pc2 pc3 pc4];
 pc=Pc(:,str2double(char(split_name(2))));
 %% history tau_s
 tau1 = -0.934;
 tau2 = -0.489;
 tau4 = 0.268;
 Tau=[tau1 tau2 0 tau4];
 tau=Tau(str2double(char(split_name(2))));
 %% trocar outport w.r.t trocar world
T_ch_tr = [Expm([0 0 tau+(11/180*pi)]') ...
    pc;0 0 0 1];%2nd arm of 03 surgical
% figure(1);
% cla;
% hold on;

%% calc and plot
pos_err = zeros(N,1);
dir_err = zeros(N,1);
% ori_err = zeros(N,1);
% dp_err = zeros(N-1,1);
% do_err = zeros(N-1,1);
qa = zeros(24,N);
Tg=zeros(4,4,N);
Tg_=zeros(4,4,N);
i = 1;
for j=1:N
if(T_m_tr(3,4,j)~=0)
    %cla;

    %test0831(1:3,1:3,i) = test0831(1:3,1:3,i)*Expm([0 0 pi]');
    T_m_tr(1:3,4,j)=T_m_tr(1:3,4,j)/1000;
    %delta using old.
    psi_ = [psi(j,2) psi(j,1)/1000 psi(j,3) -psi(j,4) psi(j,5) -psi(j,6)]';
    Tg(1:4,1:4,j) = T_ch_tr\T_m_tr(1:4,1:4,j);
    %[theta_trocar,len_trocar]=calcThetaInTrocar(psi_,seg_len,zeta,bend_in_trocar,offset_zero);

    qa(:,j) = calcActuation_dual(psi_, structure_para2,1);
%     PlotAxis(0.01,eye(4));%trocar base considering tau
%     PlotAxis(0.01,inv(T_ch_tr));%trocar world frame    
%     PlotCircle(5.0e-3,[0 0 0],[0 0 1]);
%     PlotAxis(0.005,Tg(1:4,1:4,j));%actual target 
    if(options~=1)
        psi_ = [psi_(1) psi_(2) psi_(3) -psi_(4) psi_(5) -psi_(6)]';
        config=calcConfiguration(qa(:,j),psi_,structure_para,1);
        psi_=config;
        psi_ = [psi_(1)+structure_para(9) psi_(2) psi_(3) -psi_(4) psi_(5) -psi_(6)]';
        
    end
  
        [Tall]=PlotSnake_trocar(psi_, seg_len, zeta, 0, 0, bend_in_trocar,offset_zero);  
        Tg_(1:4,1:4,j) = Tall.T_tipg;
%         PlotAxis(0.005,Tg_(1:4,1:4,j));%theoretical target
%         axis equal
%         grid on
% %         cla;
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
% disp(['average abs pos err is ' num2str(p_avg) ' mm']);
% disp(['average abs ori err is ' num2str(o_avg) ' deg']);

end
