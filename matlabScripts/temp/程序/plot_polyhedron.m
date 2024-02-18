function [S1Obj]=plot_polyhedron(V,F,FaceColor)
%V定义顶点
%F是定义顶点连接顺序，定义组成的面
S.Vertices = V;
S.Faces = F;
S.FaceVertexCData = jet(size(V,1));
S.FaceColor = FaceColor;
S1Obj=patch(S,'FaceAlpha',0.6,'LineStyle','--','LineWidth',0.5,'Visible','on' );

end
   