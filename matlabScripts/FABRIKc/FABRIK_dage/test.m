theta=linspace(-pi/2,pi/2,50);

pos=[0;0];
% pc1=planar_continuum(0,100,[0;0],eye(2),[0.8,0,0]);
% pc2=planar_continuum(0,100,pc1.endtip,eye(2),[0,0.8,0]);
% pc3=planar_continuum(0,100,pc2.endtip,eye(2),[0,0,0.8]);
n=1;
tic
for i= 1:size(theta,2)
    for j= 1:size(theta,2)
        for k= 1:size(theta,2)
            pc1=planar_continuum(theta(i),100,[0;0],eye(2),[0.8,0,0]);
            pc2=planar_continuum(theta(j),100,pc1.endtip,eye(2),[0,0.8,0]);
            pc3=planar_continuum(theta(k),100,pc2.endtip,eye(2),[0,0,0.8]);
            
            pos(:,n)=pc3.endtip;
            n=n+1;
        end
    end
end

flag=0;
num=0;
for iter=1:size(pos,2)
    [flag(iter),num(iter)]=FABRIK_without_ori(pos(:,iter));
    
end
toc