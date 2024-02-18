function f = costFunc_CWHK(x)

    [Xcw, Xhk]=getAllData;
    block_size = size(Xhk,2);
    T0 = fromX2T(x);
    pos_err = zeros(block_size,1);
    ang_err = zeros(block_size,1);
    base_HK = fromX2T(Xhk(:,20));
    base_CW = fromX2T(Xcw(:,20));
%     tt=[2     3     4     5     6     7     8     9    10    11    12    13    14    15    16 ...
%             17    18    19    20    21    22    23    24    25    27    29    30    31    32    34 ...
%             36    38    42    43    44    45    46];
%     ttt=tt([1     2     3     4     5     6     7     8     9    10    11    12    13    14    15 ...
%         16    17    18    19    20    21    22    23    24    29    30    31    32    33    34 ...
%         35    36    37]);
    for i = 1:block_size
        Tcw=base_CW\fromX2T(Xcw(:,i));
        Thk=base_HK\fromX2T(Xhk(:,i));
        dX=calcDeviationByT(T0\Tcw*T0,Thk);
        pos_err(i)=norm(dX(1:3));
        ang_err(i)=norm(dX(4:6));
    end
%     pos_err(all(pos_err==0,2),:) = [];
%     ang_err(all(ang_err==0,2),:) = [];
    f=mean(pos_err)+mean(ang_err);%mean(pos_err);%;%+
end