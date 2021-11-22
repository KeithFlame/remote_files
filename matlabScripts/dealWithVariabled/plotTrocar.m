function plotTrocar
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.11.2021
% Ver. 1.0
% as name said,drawing the trocar.
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%%

trocar_length = 20;
[~, ~, ~, ~, ~, ~, ~, ~, r_trocar,~]=getToolArmStructureParameter;
fig_resolution=256;
hold on;

[X,Y,Z] = cylinder(r_trocar,fig_resolution);
h = trocar_length;
Z = Z*h-trocar_length;
surf(X,Y,Z,'FaceAlpha',0.3,'EdgeColor','none','FaceColor',[0 1 1]);

aminor = r_trocar/2; % Torus minor radius
Rmajor = r_trocar*3/2; % Torus major radius
theta = linspace(-pi, pi, fig_resolution) ; % Poloidal angle
phi = linspace(0., 2*pi, fig_resolution) ; % Toroidal angle
[t, p] = meshgrid(phi, theta);
x = (Rmajor + aminor.*cos(p)) .* cos(t);
y = (Rmajor + aminor.*cos(p)) .* sin(t);
z = 0.*sin(p);
surf(x, y, z,'FaceAlpha',0.3,'EdgeColor','none','FaceColor',[0 1 1]);
% shading interp; 
hold off;
end