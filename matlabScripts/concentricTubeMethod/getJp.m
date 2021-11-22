function Jp=getJp(u,P,R,ds)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.18.2021
% Ver. 1.0
% input1: curvature
% input2: posistion
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

%% para
persistent Jp_per;
if(nargin==0&&~isempty(Jp_per))
    Jp=Jp_per;
    return;
end

block_size=size(u,1);
Jp=zeros(block_size,block_size);
for i = 1:block_size/3
    ui=u(:,i);
    pi=P(:,i);
    Ri=R(:,:,i);
    dsi=ds(i);
    
    %% get Jpi
    Jpi=getJpi(ui,pi,Ri,dsi);
    Jp(i:i+2,i:i+2)=Jpi;
end
Jp_per=Jp;
end


