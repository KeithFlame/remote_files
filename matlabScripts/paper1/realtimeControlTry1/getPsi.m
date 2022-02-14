function psi=getPsi(T,LS)
%
%
%
    L1=LS(1);Lr=LS(2);L2=LS(3);Lg=LS(4);
    config=IKv5(T,Lr,L2,Lg);
    if(config(2)>L1)
        config=invKine(T,L1,Lr,L2,Lg);
    end
    psi=[config(1) config(2) config(3) config(5) config(7) config(9)];
end