M=load('psi_list.log');
block_size=max(size(M));
fpath='./psi_list2.log';
fread=fopen(fpath,'w');
for i =1:block_size
    psi=M(block_size+1-i,:);
    fprintf(fread,'%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f\n', psi);
end
fclose(fread);


M=load('psi_list_noc.log');
block_size=max(size(M));
fpath='./psi_list_noc2.log';
fread=fopen(fpath,'w');
for i =1:block_size
    psi=M(block_size+1-i,:);
    fprintf(fread,'%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f\n', psi);
end
fclose(fread);