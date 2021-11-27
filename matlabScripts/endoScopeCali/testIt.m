% % function testIt
% [T,endoPsi]=getTandConfigV4;
T=Ttc2end;

x=setX0;
% gamma3=0;
% endoCur=fromPsi2Curvature(endoPsi,x);
% endoQ=fromCurvature2Movitation(endoCur,x);

endoPsi_=fromConfig2Psi(T,x,gamma3);
endoCur_=fromPsi2Curvature(endoPsi_,x);
endoQ_=fromCurvature2Movitation(endoCur_,x);

LS=x(1:4);
nS=zeros(1,6);
endoPsi_i=endoPsi_;
Tt=T;


config=zeros(1,10);
figure;
hold on;
axis equal;
view([0 0])
axis([ -50 50 -50 50 0 150]);
t=34;
for i = 1:size(T,3)
    qc=RR_reducedDimention(q,T(:,:,i),LS,nS);
    endoPsi_i(i,:)=qc';
    config(1)=endoPsi_i(i,2);config(2)=endoPsi_i(i,1);config(3)=endoPsi_i(i,3);
    config(4)=x(1);config(5)=endoPsi_i(i,2);config(6)=x(2);config(7)=endoPsi_i(i,5);config(8)=x(3);
    config(9)=endoPsi_i(i,6);config(10)=x(4);
    [Tc,~,~,~,~,~,s1,s2,s3,s4,s5]=FKfunc(config);
    Tt(:,:,i)=Tc;
    plot3(s1(1,:),s1(2,:),s1(3,:),'k',s2(1,:),s2(2,:),s2(3,:),'r',s3(1,:),s3(2,:),s3(3,:),'g',s4(1,:),s4(2,:),s4(3,:),'b',s5(1,:),s5(2,:),s5(3,:),'m','linewidth',2,'LineStyle','-');

end
for i = 1:1
%     if(endoPsi_(i,2)-endoPsi(i,2))>1.5
%         continue;
%     end
    config(1)=endoPsi_(i,1);config(2)=endoPsi_(i,2);config(3)=endoPsi_(i,3);
    config(4)=x(1);config(5)=endoPsi_(i,2);config(6)=x(2);config(7)=endoPsi_(i,5);config(8)=x(3);
    config(9)=endoPsi_(i,6);config(10)=x(4);
    [Tc,~,~,~,~,~,s1,s2,s3,s4,s5]=FKfunc(config);
    plot3(s1(1,:),s1(2,:),s1(3,:),'k',s2(1,:),s2(2,:),s2(3,:),'r',s3(1,:),s3(2,:),s3(3,:),'g',s4(1,:),s4(2,:),s4(3,:),'b',s5(1,:),s5(2,:),s5(3,:),'m','linewidth',2,'LineStyle','-');
    
%     plotCoord(Tc);
    plotCoord(T(:,:,i));
%     config(1)=endoPsi(i,1);config(2)=endoPsi(i,2);config(3)=endoPsi(i,3);
%     config(5)=endoPsi(i,4);config(7)=endoPsi(i,5);config(9)=endoPsi(i,6);
%     [Tc,~,~,~,~,~,s1,s2,s3,s4,s5]=FKfunc(config);
%     plot3(s1(1,:),s1(2,:),s1(3,:),'k',s2(1,:),s2(2,:),s2(3,:),'r',s3(1,:),s3(2,:),s3(3,:),'g',s4(1,:),s4(2,:),s4(3,:),'b',s5(1,:),s5(2,:),s5(3,:),'m','linewidth',2,'LineStyle','-.');
    Tc,T(:,:,i),
end
xlabel('X');
ylabel('Y');
zlabel('Z');
% axis equal;
view([0 0])
x=0;

% end