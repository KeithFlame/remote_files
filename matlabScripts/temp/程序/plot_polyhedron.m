function [S1Obj]=plot_polyhedron(V,F,FaceColor)
%V���嶥��
%F�Ƕ��嶥������˳�򣬶�����ɵ���
S.Vertices = V;
S.Faces = F;
S.FaceVertexCData = jet(size(V,1));
S.FaceColor = FaceColor;
S1Obj=patch(S,'FaceAlpha',0.6,'LineStyle','--','LineWidth',0.5,'Visible','on' );

end
   