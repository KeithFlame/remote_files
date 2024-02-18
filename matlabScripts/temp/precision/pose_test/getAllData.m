function [Xcw, Xhk]=getAllData

persistent xcw;
persistent xhk;
if(isempty(xcw)&&isempty(xhk))
    file_name='data_HK2.log';
    X_HK = data_processing_HK(file_name);
    X_CW = getData_CW;
    t=find(X_HK(7,:)<50);
    xcw = X_CW(:,t);
    xhk = X_HK(1:7,t);
end
Xcw = xcw;
Xhk = xhk(1:6,:);
end
