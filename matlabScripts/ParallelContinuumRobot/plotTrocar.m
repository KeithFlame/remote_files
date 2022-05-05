function plotTrocar(trocar)
% this is a function to draw the trocar
%  
% Author Keith W.
% Ver. 1.0
% Date 05.02.2022
x=trocar(1:3,:);
y=trocar(4:6,:);
z=trocar(7:9,:);
patch(x,y,z,'w','FaceAlpha',.5,'EdgeColor','none','FaceColor',[0.5 0.5 0.5]);

end

