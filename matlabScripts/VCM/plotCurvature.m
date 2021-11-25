function []=plotCurvature(u_ref)
%-----plot curvature u_ref along length
% ver p1.0
% By Yuyang Chen
% Date 20200606
%-----------------------------------------------------------------------%

sizeu=size(u_ref);
N=sizeu(2);
figure(3);hold on;
    plot3(zeros(N,1),zeros(N,1),u_ref(3,:)*1e3,'-','Color',[0.8 0.6 0.8],'LineWidth',3);
    plot3(zeros(N,1),zeros(N,1),u_ref(3,:)*1e3,'.r','MarkerSize',6);
    plot3(u_ref(1,:),u_ref(2,:),u_ref(3,:)*1e3,'-r','LineWidth',3);
    %plot3(u_fit(1,:),u_fit(2,:),u_fit(3,:)*1e3,'-b','LineWidth',3);
    %plot3(u_spl(1,:),u_spl(2,:),u_spl(3,:)*1e3,'*r','MarkerSize',10,'LineWidth',2);
for i=1:N
    %plot3([0 u_ref(1,i)],[0 u_ref(2,i)],[u_ref(3,i) u_ref(3,i)]*1e3,'-r'); 
    plotArrow([0 0 u_ref(3,i)]'*1e3,[u_ref(1,i) u_ref(2,i) 0]'); 
end
xlabel('u_x (m^-^1)');
ylabel('u_y (m^-^1)');
zlabel('length (mm)');

end