function psi = psi_random(config)
%	输出在指定构型config下的psi
    switch(config)
        case{1}%构型1
            psi=[2*pi*rand(1);2/3*pi*rand(1);60*rand(1);2*pi*rand(1)];
            psi(2)=min(psi(2),psi(3)*2/3*pi/60);
        case{2}%构型2
            psi=[2*pi*rand(1);20*rand(1);2/3*pi*rand(1);2*pi*rand(1)];
        case{3}%构型3
            psi=[2*pi*rand(1);1/2*pi*rand(1);40*rand(1);2*pi*rand(1);2/3*pi*rand(1);2*pi*rand(1)];
            psi(2)=min(psi(2),psi(3)*pi/2/40);
        case{4}%构型4   
            psi=[2*pi*rand(1);150*rand(1);1/2*pi*rand(1);2*pi*rand(1);2/3*pi*rand(1);2*pi*rand(1)];
    end
end

