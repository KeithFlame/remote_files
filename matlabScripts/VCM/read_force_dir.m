data = importdata('./0925/forcedir.txt');
data1=data.data;figure(1);hold on;
plotAxis(0.01,eye(4))
for i=1:8
    R_bC=[data1(i,5:7);data1(i,8:10);data1(i,11:13)];
    R_tC=[data1(i,17:19);data1(i,20:22);data1(i,23:25)];
    R_bB=[0 -1 0;-1 0 0;0 0 -1]';
    R_tB=R_bB*R_bC'*R_tC;
    
    
    if(i>4)
        vec(:,i)=-R_tB(:,1);
    else
        vec(:,i) = R_tB(:,1);
    end
    plotArrow([0 0 0],vec(:,i),0.05,[1 0 0]);
end
axis equal;
grid on;