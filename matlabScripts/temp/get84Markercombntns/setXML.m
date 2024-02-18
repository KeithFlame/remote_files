p1=[5.5 0
    5.5 11
    -5.5 11
    -5.5 0
    0 0];
p2=[-3.5 9.0
    0 9.0
    3.5 9.0
    -3.5 6.0
    0 6.0
    3.5 6.0
    -3.5 3.0
    0 3.0
    3.5 0];
t=combntns(1:9,3);
fread=fopen("marker_decal.xml",'w');
fprintf(fread,'<?xml version="1.0" encoding="UTF-8"?>\n');
for i = 1:14
    str = ['<point id="',num2str(i-5),'">\n'];
    fprintf(fread,str);
    if(i<6)
        str = ['    <data>',num2str(p1(i,1)),'</data>\n'];
        fprintf(fread,str);
        str = ['    <data>',num2str(p1(i,2)),'</data>\n'];
        fprintf(fread,str);
    else
        p0=p2(i-5,:);
        str = ['        <data>',num2str(p0(1)),'</data>\n'];
        fprintf(fread,str);
        str = ['        <data>',num2str(p0(2)),'</data>\n'];
        fprintf(fread,str);
    end
    str = '</point>\n';
    fprintf(fread,str);
end