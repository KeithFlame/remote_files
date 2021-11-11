function [T1,Config,qa,xN_init]=getTandConfigV2_1110(serials)

fpath=['../systemCali/test1102/pose/Ttrocar_marker_data_',serials,'.mat'];
t=load(fpath);
T_m_tr=t.Ttrocar_marker;
fpath=['../systemCali/test1102/psi/Psi_actual_',serials,'.mat'];
t=load(fpath);
Psi=t.Psi';
fpath=['../systemCali/test1102/qa/Qa_',serials,'.mat'];
t=load(fpath);
Qa=t.Qa';
split_name = regexp(serials, '_', 'split');
optimal_path='../systemCali/test1102/optimal_res.mat';
t=load(optimal_path);
optimal_res=t.optimal_res;
[~,po]=min(abs(optimal_res(:,1)-str2double(char(split_name(1)))));
xN_init=optimal_res(po,2:12);
%% trocar channel positions
 pc1= [0.0  -7.85  0.0 ]'*1e-3;
 pc2= [7.66 -1.7   0.0 ]'*1e-3;
 pc3= [0.0   5.8 -15.0 ]'*1e-3;
 pc4=[-7.66 -1.7   0.0 ]'*1e-3;
 Pc=[pc1 pc2 pc3 pc4]*1e3;
 pc=Pc(:,str2double(char(split_name(2))));
 %% history tau_s
 tau1 = -0.934;
 tau2 = -0.489;
 tau4 = 0.268;
 Tau=[tau1 tau2 0 tau4];
 tau=Tau(str2double(char(split_name(2))));
 %%
T_ch_tr = [Expm([0 0 tau+(11/180*pi)]') ...
    pc;0 0 0 1];
T1=zeros(4,4,size(T_m_tr,3));
for i=1:size(T1,3)
    if(T_m_tr(3,4,i)~=0)
        T1(:,:,i)=T_ch_tr\T_m_tr(:,:,i);
    else
        T1(:,:,i)=T_m_tr(:,:,i);
    end
end
Config = Psi;
t=Config(:,2);
Config(:,2)=Config(:,1);
Config(:,1)=t;
Config(:,2)=Config(:,2)-sum(xN_init(2:3));
qa=Qa;

end