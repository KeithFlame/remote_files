function f = costFunc_CoordHK(x)

    [Xcoord, Xhk]=getAllData_2;
    block_size = size(Xcoord,2);
    T0 = fromX2T(x);
    pos_err = zeros(block_size,1);
    ang_err = zeros(block_size,1);
    base_HK = fromX2T(Xhk(:,1));
    base_Coord = fromX2T(Xcoord(:,1));
    for i = 1:block_size
        Tcoord=base_Coord\fromX2T(Xcoord(:,i));%
        Thk=base_HK\fromX2T(Xhk(:,i));%
        dX=calcDeviationByT(Tcoord,T0\Thk*T0);
        pos_err(i)=norm(dX(1:3));
        ang_err(i)=norm(dX(4:6));
    end
    f=mean(pos_err(2:end))+mean(ang_err(2:end));%mean(pos_err);%;%+
end