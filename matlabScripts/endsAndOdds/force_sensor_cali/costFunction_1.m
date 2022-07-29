function f = costFunction_1(x)
    F0 = [200	500	700	1000	1200	1500	1700	2000];
    vol = [360	585	715	1200	1375	1570	1680	1810
        350	550	795	1200	1360	1530	1600	1700
        360	580	645	1180	1360	1455	1620	1720
        ];
    a = x(1);
    b = x(2);
    omiga1 = x(3);
    u0 = x(4);

%     u0 = 3.285;
    f0 = 0;
    block_size = size(F0,2);
    block_size2 = size(vol,1);
    for i = 1 : block_size
        u = u0*omiga1/(omiga1+a*F0(i)^b);
        for j = 1 : 1
            f0 = f0 + abs(vol(j,i) - u);
        end
    end
    f = f0;
end