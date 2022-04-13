% for cube
close all;
h = figure;
ax = axes('Parent',h);      % 以h为父对象框架下，创建坐标，并且该坐标成为当前坐标
ax.XAxis.Visible = 'off';   % 设置y轴不可见
ax.YAxis.Visible = 'off';   % 设置y轴不可见
ax.ZAxis.Visible = 'off';   % 设置y轴不可见

hold on;
% % 
% % initial_point = [0 0 0];
% % end_point = initial_point+[4 0.01 3];
% % plotCuboid(initial_point,end_point,'#fab');
% % 
% % initial_point = [4 0 0];
% % end_point = initial_point+[0 3 3];
% % plotCuboid(initial_point,end_point,'#a9c');
% % 
% % initial_point = [0 0 3];
% % end_point = initial_point+[4 3 0];
% % plotCuboid(initial_point,end_point,'#ff0');
w=8;h=3;v=3;h_=1/4*sqrt(3);w_=2;
X=[0 w w 0];Y=[0 0 0 0];Z=[0 0 v v];
fill3(X,Y,Z,[ 0.9569    0.6784    0.6941]);
X=[0 w w+w_ w_];Y=[0 0 h*h_ h*h_];Z=[v v v v];
fill3(X,Y,Z,[ 1    1    0]);
X=[w w+w_ w+w_ w];Y=[0 h*h_ h*h_ 0];Z=[0 0 v v];
fill3(X,Y,Z,[0.6314    0.5961    0.7647]);
for i =1:3
    line([i*2,i*2],[0,0],[0,v],'Color','black');
    line([i*2,i*2+w_],[0,h*h_],[v,v],'Color','black');
end
for i =1:2
    line([0,w],[0,0],[i,i],'Color','black');
    line([2/3*i,2/3*i+w],[h*h_*i/3,h*h_/3*i],[v,v],'Color','black');
    line([w,w+w_],[0,h*h_],[i,i],'Color','black');
    line([2/3*i+w,2/3*i+w],[h*h_/3*i,h*h_/3*i],[0,v],'Color','black');
end

view([30 15])


function plotCuboid(start_point,final_point,face_color)
%% 根据三维空间中起点坐标和终点坐标绘制长方体
%输入start_point：             起点坐标，如[1，1，1];
%输入final_point：             终点坐标，如[5，6，7];
%输出：                        长方体
%% 根据起点和终点，计算长方体的8个的顶点
vertexIndex=[0 0 0;0 0 1;1/sqrt(2) 1/sqrt(2) 0;1/sqrt(2) 1/sqrt(2) 1;
    1 0 0;1 0 1;1+1/sqrt(2) 1/sqrt(2) 0;1+1/sqrt(2) 1/sqrt(2) 1];
cuboidSize=final_point-start_point;             %方向向量
vertex=repmat(start_point,8,1)+vertexIndex.*repmat(cuboidSize,8,1);
%% 定义6个平面分别对应的顶点
facet=[1 2 4 3;1 2 6 5;1 3 7 5;2 4 8 6;3 4 8 7;5 6 8 7];
%% 定义8个顶点的颜色，绘制的平面颜色根据顶点的颜色进行插补
color=[0;0;0;0;1;1;1;1];
%% 绘制并展示图像
patch('Vertices',vertex,'Faces',facet,'FaceVertexCData',color,'FaceColor',face_color,'FaceAlpha',1);
view([1,1,1]);
xlabel('X');
ylabel('Y');
zlabel('Z');
% grid on
%% 设置xyz显示范围
xmin=min(vertex(:,1))-1;
xmax=max(vertex(:,1))+1;
ymin=min(vertex(:,2))-1;
ymax=max(vertex(:,2))+1;
zmin=min(vertex(:,3))-1;
zmax=max(vertex(:,3))+1;
axis equal;
% axis([xmin xmax ymin ymax zmin zmax]) 
end