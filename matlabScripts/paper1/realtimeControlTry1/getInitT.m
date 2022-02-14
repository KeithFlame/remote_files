function [T,S]=getInitT(config)
%
%
%
    trocar = getStructurePara;
    c=trocar.c;
    lsi=config(1);
    l1i=config(2);
    thetasi=config(3);
    theta1i=config(4);
    d=config(5);
    delta1=config(6);

    T_base=eye(4);T_base(3,4)=-d;
    phi=0;
    T_base(1:3,1:3)=[cos(phi) -sin(phi) 0;sin(phi) cos(phi) 0;0 0 1];
    L=lsi;theta=thetasi;delta=delta1;
    [T0,~]=plotSeg(L,theta,delta,T_base);
    L=l1i;theta=theta1i;delta=delta1;
    [T1,~]=plotSeg(L,theta,delta,T0);
    
    
    tem_norm=norm(T1(1:2,4));
    if(tem_norm>c/2)
        T_base(1:2,4)=(c/2-tem_norm)/tem_norm*T1(1:2,4);
    end
    L=Lsi;theta=thetasi;delta=delta1;
    [T0,S0]=plotSeg(L,theta,delta,T_base);
    L=L1i;theta=theta1i;delta=delta1;
    [T1,S1]=plotSeg(L,theta,delta,T0);
    T=T1;
    S=[S0,S1];
end