function [Xcoord, Xhk]=getAllData_2

persistent xcoord;
persistent xhk;
if(isempty(xcoord)&&isempty(xhk))
    file_name='data_HK7.log';
    X_HK = data_processing_HK(file_name);
    X_Coord = getData_Coord;
    
    t=find(X_HK(7,:)<50);
    xcoord = X_Coord(:,t);
    [X_HK,~] = getT_CW;
    xhk = X_HK(1:6,t);
end
Xcoord = xcoord;
Xhk = xhk(1:6,:);
end
