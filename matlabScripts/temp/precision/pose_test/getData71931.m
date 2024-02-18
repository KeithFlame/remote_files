function [C7, C19, C31,marker_ID0]=getData71931(file,flag)
if(nargin == 1)
    flag = 1;
end
if(flag==1)
    marker_ID=[7 19 31];
elseif(flag==2)
    marker_ID=[4 14 40];
end
marker_ID1 = ['SBM',num2str(marker_ID(1))];
marker_ID2 = ['SBM',num2str(marker_ID(2))];
marker_ID3 = ['SBM',num2str(marker_ID(3))];


str = ['F:\tem\0906\0906\',file,'.txt'];
[a1,a2,a3,a4]=textread(str,'%s%f%f%f','headerlines',0);

size_max = size(a1,1)+1;
C7 = zeros(8,3);C19 = zeros(8,3);C31 = zeros(8,3);
i7=1;i19=1;i31=1;
for i = 1:size(a1,1)
    tem = strsplit(cell2mat(a1(i)), '_');
    if(strcmp(cell2mat(tem(1)), marker_ID1) &&i7<size_max)
        C7(i7,:)=[a2(i) a3(i) a4(i)];
        i7=i7+1;
    elseif(strcmp(cell2mat(tem(1)), marker_ID2)&&i19<size_max)
        C19(i19,:)=[a2(i) a3(i) a4(i)];
        i19 = i19 +1;
    elseif(strcmp(cell2mat(tem(1)), marker_ID3)&&i31<size_max)
        C31(i31,:)=[a2(i) a3(i) a4(i)];
        i31 = i31 +1;
    end
end
marker_ID0 = cell2mat(strsplit(cell2mat(a1(1)), '_'));
end