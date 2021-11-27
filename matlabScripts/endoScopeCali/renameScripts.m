path ='E:\MatlabScripts\coupleSolution\autoCali_20210221\newCost\';
saveFile='123';
mkdir (saveFile);
Files= dir(strcat(path,'*.m'));
savePath=[path saveFile '\'];
for i=1:length(Files)    
    x1=Files(i).name;        
    x3=[x1 '.log'];
    copyfile([path x1],[savePath x3]);
end

