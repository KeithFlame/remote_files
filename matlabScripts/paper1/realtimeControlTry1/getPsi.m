function psi=getPsi(T)
%
%
%
    tro=getStructurePara;
    LS=tro.LS*1000;
    L1=LS(1);Lr=LS(2);L2=LS(3);Lg=LS(4);
    T(1:3,4)=T(1:3,4)*1000;
    config=IKv5(T,Lr,L2,Lg);
    psi=[config(1) config(4)+Lr+L2 config(3) config(5) config(7) config(9)];
    if(config(4)>L1)
        config=invKine(T,L1,Lr,L2,Lg);
        psi=[config(1) config(2)+L1+Lr+L2 config(3) config(5) config(7) config(9)];
    end
    psi(2)=psi(2)/1000;
end