function [T1,Config,qa]=getTandConfigV2_1110(serials)

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
 %%
T_ch_tr = [Expm([0 0 tau+(11/180*pi)]') ...
    pc;0 0 0 1];
T1=zeros(4,4,size(T_m_tr,3));
for i=1:size(T1,3)
    if(T1(3,4,i)~=0)
        T1(:,:,i)=T_ch_tr\T_m_tr(:,:,i);
    else
        T1(:,:,i)=T_m_tr(:,:,i);
    end
end
Config = Psi;
t=Config(:,2);
Config(:,2)=Config(:,1);
Config(:,1)=t;
qa=Qa;

end