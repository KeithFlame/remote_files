%---------calc qa--------------------%
% calculating qa.
% ----Info
% By Keith W.
% Date 20201110
% Ver c1.0
% 
%-------------------------------------------------------------------------%
name ='70_4';

split_name = regexp(name, '_', 'split');
optimal_path='./test1102/optimal_res.mat';
t=load(optimal_path);
optimal_res=t.optimal_res;
[~,po]=min(abs(optimal_res(:,1)-str2double(char(split_name(1)))));
structure_para=optimal_res(po,2:12);

psi_path=['./test1102/psi/Psi_actual_',name,'.mat'];
t=load(psi_path);
Psi_actual=t.Psi';

block_size=size(Psi_actual,1);
Qa=zeros(24,block_size);
for j =1:block_size
    psi=Psi_actual(j,:);
    psi_ = [psi(2) psi(1)/1000 psi(3) -psi(4) psi(5) -psi(6)]';
    [qa]=calcActuation_dual(psi_,structure_para,1);
    Qa(:,j)=qa;
end

fname=['./test1102/qa/Qa_',name];
save(fname,'Qa');