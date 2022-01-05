function [Tend,Psi]=getEnd(name)

temp=1;

split_name = regexp(name, '_', 'split');
pose_path=['../../systemCali/test1102/pose/Ttrocar_marker_data_',name,'.mat'];
t=load(pose_path);
T_m_tr=t.Ttrocar_marker;
t=load(['../../systemCali/test1102/psi/Psi_actual_',name,'.mat']);
Psi=t.Psi;

if(temp)
    pose_path=['../../systemCali/test1206/pose/Ttrocar_marker_',name,'.mat'];
    t=load(pose_path);
    T_m_tr=t.Ttrocar_marker;
    t=load(['../../systemCali/test1206/psi/Psi_actual_',name,'.mat']);
    Psi=t.Psi_actual;
end


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


block_size=max(size(T_m_tr));
Tg=zeros(4,4,block_size);
for j=1:block_size
    if(T_m_tr(3,4,j)==0)
        continue;
    end
    T_m_tr(1:3,4,j)=T_m_tr(1:3,4,j)/1000;
    Tg(1:4,1:4,j) = T_ch_tr\T_m_tr(1:4,1:4,j);
end
Tend=Tg;
end