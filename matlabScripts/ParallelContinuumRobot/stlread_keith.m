function [x, y, z, varargout] = stlread_keith(filename)

%转发\作者信息：山东大学计算机基地Frankdura
%时间：2018\5\1
% This function reads an STL file in binary format into matrixes X, Y and
% Z, and C.  C is optional and contains color rgb data in 5 bits.  
%
% USAGE: [x, y, z, c] = stlread(filename);
%
% To plot use patch(x,y,z,c), or patch(x,y,z)
%
% Written by Doron Harlev

if nargout>4
    error('Too many output arguments')
end
use_color=(nargout==4);

fid=fopen(filename, 'r'); %Open the file, assumes STL Binary format.
if fid == -1 
    error('File could not be opened, check name or path.')
end

ftitle=fread(fid,80,'uchar=>schar'); % Read file title
num_facet=fread(fid,1,'int32'); % Read number of Facets

fprintf('\nTitle: %s\n', char(ftitle'));
fprintf('Num Facets: %d\n', num_facet);

% Preallocate memory to save running time
x=zeros(3,num_facet); y=zeros(3,num_facet); z=zeros(3,num_facet);
if use_color
    c=uint8(zeros(3,num_facet));
end

h = waitbar(0,'Please wait...');
for i=1:num_facet,
    norm=fread(fid,3,'float32'); % normal coordinates, ignored for now
    ver1=fread(fid,3,'float32'); % vertex 1
    ver2=fread(fid,3,'float32'); % vertex 2
    ver3=fread(fid,3,'float32'); % vertex 3
    col=fread(fid,1,'uint16'); % color bytes
    if (bitget(col,16)==1 & use_color)
        r=bitshift(bitand(2^15-1, col),-10);
        g=bitshift(bitand(2^10-1, col),-5);
        b=bitand(2^5-1, col);
        c(:,i)=[r; g; b];
    end
    x(:,i)=[ver1(1); ver2(1); ver3(1)]; % convert to matlab "patch" compatible format
    y(:,i)=[ver1(2); ver2(2); ver3(2)];
    z(:,i)=[ver1(3); ver2(3); ver3(3)];
    if mod(i,floor(num_facet/10))==0
        waitbar(i/num_facet,h);
    end
end
if use_color
    varargout(1)={c};
end
fclose(fid);
close(h);


% % % % % % % fp = fopen(filename,'rb');
% % % % % % % src = fread(fp,'uint8=>uint8');
% % % % % % % fclose(fp);
% % % % % % % 
% % % % % % % %% 二、提取有效信息
% % % % % % % % 提取数据长度信息（四字节无符号整形）
% % % % % % % len = typecast(src(81:84),'uint32');
% % % % % % % 
% % % % % % % % 提取三角片信息（[48有效字节+2填充字节]*len）
% % % % % % % data = reshape(src(85:end),[50,len]);
% % % % % % % data(end-1:end,:) = [];
% % % % % % % 
% % % % % % % % 类型转换(float*12*len)
% % % % % % % dataf = typecast(data(:),'single');
% % % % % % % dataf = reshape(dataf,[12,len]);
% % % % % % % 
% % % % % % % %% 三、获取v、f、n
% % % % % % % % 获取v,f,n（注意MATLAB是列优先的，所以必须按下列方式写）
% % % % % % % n = dataf(1:3,:)';
% % % % % % % v = reshape(dataf(4:end,:),[3,len*3])';
% % % % % % % f = reshape((1:len*3)',[3,len])';
% % % % % % % 
% % % % % % % % 去除重复顶点
% % % % % % % [v, ~, indexn] =  unique(v, 'rows');
% % % % % % % f = indexn(f);
% % % % % % % x=n;y=v;z=f;
