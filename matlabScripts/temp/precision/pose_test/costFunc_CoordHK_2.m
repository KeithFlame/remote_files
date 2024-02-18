function f = costFunc_CoordHK_2(x)

    [Xcoord, Xhk]=getAllData_2;
    Xcoord = fliplr(Xcoord);
    block_size = size(Xcoord,2);
    T0 = fromX2T(x);
    pos_err = zeros(block_size,1);
    ang_err = zeros(block_size,1);
    base_HK = fromX2T(Xhk(:,end));
    base_Coord = fromX2T(Xcoord(:,end));
    for i = 28:block_size*2
        Tcoord=base_Coord\fromX2T(Xcoord(:,i-27));%
        Thk=base_HK\fromX2T(Xhk(:,i));%
        dX=calcDeviationByT(Tcoord,T0\Thk*T0);
        pos_err(i-block_size)=norm(dX(1:3));
        ang_err(i-block_size)=norm(dX(4:6));
    end
    f=mean(pos_err(2:end))+mean(ang_err(2:end));%mean(pos_err);%;%+
end