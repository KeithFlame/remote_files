t=load('../test1206/psi/Psi_actual.mat');
Psi=t.Psi;
for i =2:2:198
    Psi(i,:)=Psi(i+2,:);
end
fop=fopen('batch_psi_2.data','w');
for i =1:size(Psi,1)
    fprintf(fop,"%f,%f,%f,%f,%f,%f,\n",Psi(i,:));
end
fclose(fop);