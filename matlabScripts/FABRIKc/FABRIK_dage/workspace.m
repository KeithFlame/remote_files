num=80;
theta1=linspace(0,pi,num);
theta2=linspace(-pi,pi,num);

n=1;
for iter1=1:num
     for iter2=1:num


%                     config(:,n)=[theta1(iter1);theta2(iter2);delta2(iter4);delta1(iter3)];
%                     seg1=rigid_seg(length1(iter1),[0;0;0],rotx(0),[17 63 103]/255);
                    seg1=planar_continuum(theta1(iter1),100,[0;0],eye(2),[0.8,0,0]);
%                     seg3=rigid_seg(20,seg2.endtip,seg2.endrotation,[64 138 230]/255);
                    seg2=planar_continuum(theta2(iter2),100,seg1.endtip,seg1.endrotation,[0,0.8,0]);
%                     seg5=rigid_seg(20,seg4.endtip,seg4.endrotation,[127 255 170]/255);
                     
                     target(:,n)=seg2.endtip;
                     target_orientation=seg2.endrotation;
                     
                     theta1_n(n)=theta1(iter1);
                     theta2_n(n)=theta2(iter2);

                     n=n+1;
                      

     end
end
 
 
shp = alphaShape(target(1,:)',target(2,:)',4);
plot(shp)
axis equal