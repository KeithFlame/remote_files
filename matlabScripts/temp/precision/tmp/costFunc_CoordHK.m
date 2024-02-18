function f = costFunc_CoordHK(x)

    [Xcoord, Xhk]=getAllData_2;
    block_size = size(Xcoord,2);
    T0 = fromX2T(x);
    pos_err = zeros(block_size,1);
    ang_err = zeros(block_size,1);
    tem = 64;
    base_HK = fromX2T(Xhk(:,tem));
    base_Coord = fromX2T(Xcoord(:,tem));
    for i = 1:block_size
        Tcoord=base_Coord\fromX2T(Xcoord(:,i));%
        Thk=base_HK\fromX2T(Xhk(:,i));%
        dX=calcDeviationByT(Tcoord,T0\Thk*T0);
        pos_err(i)=norm(dX(1:3));
        ang_err(i)=norm(dX(4:6));
    end
    pos_err(tem)=0;ang_err(tem)=0;
    pos_err(all(pos_err==0,2),:) = [];
    ang_err(all(ang_err==0,2),:) = [];
    f=mean(pos_err)+mean(ang_err);%mean(pos_err);%;%+
end