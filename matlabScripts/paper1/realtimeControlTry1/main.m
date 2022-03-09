block_size=1200;
psi_width=6;
T=zeros(4,4,block_size);
z=0.08;
r=0.04;
for i =1:block_size/6
    T(:,:,i)=eye(4);
    T(1,4,i)=(i-1)/(block_size/6)*r;
    T(2,4,i)=-r;
    T(3,4,i)=z;
end
for i =(block_size/6+1):block_size/2
    T(:,:,i)=eye(4);
    T(2,4,i)=(i-(block_size/6+1))/(block_size/3)*2*r-r;
    T(1,4,i)=r;
    T(3,4,i)=z;
end
for i =(block_size/2+1):block_size/3*2
    T(:,:,i)=eye(4);
    T(1,4,i)=-(i-(block_size/2+1))/(block_size/6)*r+r;
    T(2,4,i)=r;
    T(3,4,i)=z;
end
for i =(block_size/3*2+1):block_size
    T(:,:,i)=eye(4);
    T(2,4,i)=-(i-(block_size/3*2+1))/(block_size/3)*2*r+r;
    T(1,4,i)=0;
    T(3,4,i)=z;
end
P=reshape(T(1:3,4,:),[3 block_size]);
figure;
plot3(P(1,:)',P(2,:)',P(3,:)','*');

Psi1=zeros(psi_width,block_size);
Psi2=zeros(psi_width,block_size);
grid on;hold on;axis equal;view([90 90]);

tic;
for i =1:block_size
    temT=T(:,:,i);
    [psi,S]=InverseK(temT);
    Psi1(:,i)=psi';
    psi=getPsi(temT);
    Psi2(:,i)=psi';
    plot3(S(1,:),S(2,:),S(3,:),'-');
end
toc;
fpath='./psi_list/psi_list.log';
fread=fopen(fpath,'w');

tro=getStructurePara;
LS=tro.LS;
for i =1:block_size
    psi=Psi1(:,i)';
    psi=psi*180/pi;
    psi(2)=psi(2)/180*pi*1000-(LS(2)+LS(3))*1000;
    t=psi(1);
    psi(1)=psi(2);
    psi(2)=t;
    fprintf(fread,'%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f\n', psi);
end
fclose(fread);

fpath='./psi_list/psi_list_noc.log';
fread=fopen(fpath,'w');

tro=getStructurePara;
LS=tro.LS;
for i =1:block_size
    psi=Psi2(:,i)';
    psi=psi*180/pi;
    psi(2)=psi(2)/180*pi*1000-(LS(2)+LS(3))*1000;
    t=psi(1);
    psi(1)=psi(2);
    psi(2)=t;
    fprintf(fread,'%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f\n', psi);
end
fclose(fread);