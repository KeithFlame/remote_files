

function [T,Config]=getTandConfig
Tfilename = join([getenv('VSARMCALIBPATH'), '\T.log']);
psifilename = join([getenv('VSARMCALIBPATH'), '\psi.log']);
M = load(Tfilename);
Cf = load(psifilename);
T=zeros(4,4,size(M,1));
for i = 1:size(M,1)
    T(:,:,i)=[M(i,1:4); M(i,5:8); M(i,9:12); M(i,13:16)]';
end
% Cf(:,2)=Cf(:,2)-20;
% Cf(:,4)=-Cf(:,4);Cf(:,6)=-Cf(:,6);
Config=Cf;



end