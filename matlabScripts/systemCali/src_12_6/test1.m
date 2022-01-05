close,clear,clc;
%%
name='59_2';
[T0,T1,T2]=getPose(name);
getPortChannel(name);
t=load('../test1206/psi/Psi_actual.mat');
Psi=t.Psi;
fp=['../test1206/',name,'/SP.mat'];

t=load(fp);
SP=t.SP;
% figure;
hold on; grid on; axis equal;

% selected=13;
ang_err_all=zeros(200,1);
pos_err_all=zeros(200,1);

for selected=187:2:200
    T_t=T0(:,:,selected);
    [Tall, dis_err, ang_err]=plot_snake(T_t,Psi(selected,:),SP);
    ang_err_all(selected)=ang_err;
    pos_err_all(selected)=dis_err;
    view([0 45]);
    cla;
end
ang_err_all(all(ang_err_all==0,2),:)=[];
pos_err_all(all(pos_err_all==0,2),:)=[];
[mean(pos_err_all),mean(ang_err_all)],
% % % % T.T_tipg-