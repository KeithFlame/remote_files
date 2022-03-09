function [T,Te,S]=getInitT(config)
%
%
%
    trocar = getStructurePara;
    c=trocar.c;
    LS=trocar.LS;
    Lr=LS(2);L2=LS(3);Lg=LS(4);
    lsi=config(1);
    l1i=config(2);
    lso=config(3);
    l1o=config(4);    
    thetasi=config(5);
    theta1i=config(6);
    thetaso=config(7);
    theta1o=config(8);
    d=config(9);
    delta1=config(10);
    
    KP=setPsi_U;
    psi=KP.psi;
    phi=psi(1);theta2=psi(5);delta2=psi(6);

    T_base=eye(4);T_base(3,4)=-d;
    
    T_base(1:3,1:3)=[cos(phi) -sin(phi) 0;sin(phi) cos(phi) 0;0 0 1];
    L=lsi;theta=thetasi;delta=delta1;
    [T0,~]=plotSeg(L,theta,delta,T_base);
    L=l1i;theta=theta1i;delta=delta1;
    [T1,~]=plotSeg(L,theta,delta,T0);
    
    
    tem_norm=norm(T1(1:2,4));
    if(tem_norm>c/2)
        T_base(1:2,4)=(c/2-tem_norm)/tem_norm*T1(1:2,4);
    end
    L=lsi;theta=thetasi;delta=delta1;
    [T0,S0]=plotSeg(L,theta,delta,T_base);
    L=l1i;theta=theta1i;delta=delta1;
    [T1,S1]=plotSeg(L,theta,delta,T0);
    T=T1;
    
    L=lso;theta=thetaso;delta=delta1;
    [T2,S2]=plotSeg(L,theta,delta,T1);
    L=l1o;theta=theta1o;delta=delta1;
    [T3,S3]=plotSeg(L,theta,delta,T2);
    L=Lr;theta=0;delta=0;
    [T4,S4]=plotSeg(L,theta,delta,T3);
    L=L2;theta=theta2;delta=delta2;
    [T5,S5]=plotSeg(L,theta,delta,T4);
    L=Lg;theta=0;delta=0;
    [T6,S6]=plotSeg(L,theta,delta,T5);
    Te=T6;
    S=[S0,S1,S2,S3,S4,S5,S6];
end