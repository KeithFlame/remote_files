function Tall=plot_snake(Tm,psi,name)

split_name = regexp(name, '_', 'split');

optimal_path='./test1102/optimal_res.mat';
t=load(optimal_path);
optimal_res=t.optimal_res;
[~,po]=min(abs(optimal_res(:,1)-str2double(char(split_name(1)))));
structure_para2=optimal_res(po,2:12);
structure_para=structure_para2;

seg_len = structure_para(1:4)'*1e-3;
bend_in_trocar = -structure_para(11)*1e-3;
offset_zero = structure_para(10)*1e-3;
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

%%
Tm(1:3,4)=Tm(1:3,4)/1000;
psi_ = [psi(2) psi(1)/1000 psi(3) -psi(4) psi(5) -psi(6)]';
Tg(1:4,1:4) = T_ch_tr\Tm(1:4,1:4);
hold on;
PlotAxis(0.01,eye(4));%trocar base considering tau
PlotAxis(0.01,inv(T_ch_tr));%trocar world frame    
PlotCircle(5.0e-3,[0 0 0],[0 0 1]);
PlotAxis(0.005,Tg);%actual target 
Tall=PlotSnake_trocar(psi_, seg_len, zeta, 0, 0, bend_in_trocar,offset_zero);
PlotAxis(0.005,Tall.T_tipg);%theoretical target
if(Tg(3,4)==0)
    fprintf('NULL matrix!\n');
else
dis_err=norm(Tall.T_tipg(1:3,4)-Tg(1:3,4))*1000;
ang_err=acosd(dot(Tall.T_tipg(1:3,3),Tg(1:3,3)));


err=[dis_err,ang_err];
fprintf("pos_err: %f, ang_err: %f\n",err);
end

end