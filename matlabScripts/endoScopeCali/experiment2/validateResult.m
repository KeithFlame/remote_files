function res=validateResult(M)
% M is a 4*27 Matrix

% M=load('pose.log');
P=zeros(size(M,3),3);
for i =1:size(M,1)
%     temM=reshape(M(i,:),[3 9]);
    temM=M(1:3,4,i)';
    P(i,:)=temM;
end

ratio=0.5;
arm_id_file = [getenv('VSARMCALIBPATH'), '\arm_id.log'];
arm_id = load(arm_id_file);
map_path = ['map', '\', num2str(arm_id) ];
camaraP_path = [map_path, '\', 'camaraP.log'];
camaraP = load(camaraP_path);
para=camaraP(3);para=1;
O=[0 0 0];
O1=[16*(1-ratio) 9*(1-ratio) norm([16 9])];
O2=[16*(1-ratio) -9*(1-ratio) norm([16 9])];
O3=[-16*(1-ratio) -9*(1-ratio) norm([16 9])];
O4=[-16*(1-ratio) 9*(1-ratio) norm([16 9])];

% after opti
Q1=[P(1,1:2)*para P(1,3)];% 06--->01
Q2=[P(2,1:2)*para P(2,3)];% 08--->03
Q3=[P(3,1:2)*para P(3,3)]; % 07--->02
Q4=[P(4,1:2)*para P(4,3)]; % 09--->04

d1 = sin(acos(dot(O1-O,Q1-O)/norm(O1-O)/norm(Q1-O)))*norm(Q1-O);
d2 = sin(acos(dot(O2-O,Q2-O)/norm(O2-O)/norm(Q2-O)))*norm(Q2-O);
d3 = sin(acos(dot(O3-O,Q3-O)/norm(O3-O)/norm(Q3-O)))*norm(Q3-O);
d4 = sin(acos(dot(O4-O,Q4-O)/norm(O4-O)/norm(Q4-O)))*norm(Q4-O);

res=[d1 d2 d3 d4];


%% 2
% LS=[18 8 30 19];
% M=load('T.log');
% psi=load('psi.log');
% psi0=psi(1,:);

% % get init marker config
% M0=mean(M(1:3,:));
% T=reshape(M0,[4  4]);  % !!!!warning Transpose
% Tnew=T';
% Tnew(3,4)=new(3,4)+sum(LS);

% % get final error in marker coordinate pose-error
% Ttest=zeros(4,4,size(M,1)-1);
% Tmeasure=zeros(4,4,size(M,1)-1);
% Pcompare=zeros(size(M,1)-1,3);
% error=zeros(size(M,1)-1);
% j=1;
% for i =4:3:size(M,1)
%     psi0=psi(j+1,:);
%     M0=mean(M(i:i+3,:));
%     TM0=reshape(M0,[4  4]); % !!!!warning Transpose
%     config=[psi0(1) psi0(2) psi0(3) LS(1) psi0(4) LS(2) psi0(5) LS(3) psi0(6) LS(4)];
%     Tend=FKfunc(config);
%     Ttest(:,:,j)=inv(Tnew)*Tend;
%     Tmeasure(:,:,j)=TM0;
%     Pcompare(j,:)=(Ttest(1:3,4,j)-Tmeasure(1:3,4,j))';
%     error(j)=norm(Pcompare);
%     j = j + 1;
% end

% res=error;
end
