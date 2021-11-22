function plotTorus(Lstem_l, r_Lstem,theta_Lstem,seg_color, T_config)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.11.2021
% Ver. 1.0
% as name said,drawing the torus.
% input1: curve radius.
% input2: curved cylinder's redius.
% input3: curve angle.
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%% para
fig_resolution=256;  % figure resolution
% seg_color=100;  % continuum's color. 
if(nargin==4)
    T_config=eye(4);
end

if (theta_Lstem==0)
    %% clinder
    [x,y,z] = cylinder(r_Lstem,fig_resolution);
    h = Lstem_l;
    z = z*h;
else
    %% torus
    Lstem_r=Lstem_l/theta_Lstem;
    r=linspace(0, theta_Lstem, fig_resolution);
    st=linspace(0, 2*pi, fig_resolution);
    [u, v]=meshgrid(r, st);
    x=-((Lstem_r+r_Lstem*cos(v)).*cos(u)-Lstem_r);
    z=(Lstem_r+r_Lstem*cos(v)).*sin(u);
    y=r_Lstem*sin(v);
end
width_block=size(x,2);
height_bloack=size(x,1);
lay3=zeros(height_bloack,width_block,3);
lay_used=zeros(height_bloack,width_block,3);
lay3(:,:,1)=x;
lay3(:,:,2)=y;
lay3(:,:,3)=z;
for j =1:height_bloack
    for i =1:width_block
        temP= [lay3(j,i,1) lay3(j,i,2) lay3(j,i,3) 1]';
        tem=T_config*temP;
        lay_used(j,i,1)=tem(1);
        lay_used(j,i,2)=tem(2);
        lay_used(j,i,3)=tem(3);
    end
end
hold on;
 
surf(lay_used(:,:,1),lay_used(:,:,2),lay_used(:,:,3),'FaceAlpha',0.1,'EdgeColor','none','FaceColor',seg_color);
% shading interp;

end