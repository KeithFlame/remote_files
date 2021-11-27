function f=costFuncV2(x)
    global Tt;
    global Cf;
    xN=x;
    xN(5)=x(5)/10;xN(6)=x(6)/10;
    xN(7)=x(7)/10;
    xN(8)=x(8)*10;xN(9)=x(9)/100;
    x0=setX0;
    
    x0N=x0;
%     [Ttc2end,endoPsi]=getTandConfig;
    Ttc2end=Tt;
    endoPsi=Cf;
    se=[1 2 3   4:20 ]; % 
    Ttc2end=Ttc2end(:,:,se);
    endoPsi=endoPsi(se,:);
     
    gamma_a=xN(7);
    gamma_g=xN(9);
    Tbase21b=[cos(gamma_a) -sin(gamma_a) 0 0
    sin(gamma_a) cos(gamma_a) 0 0
    0 0 1 0
    0 0 0 1];
    T2e2camera=[cos(gamma_g) -sin(gamma_g) 0 0
    sin(gamma_g) cos(gamma_g) 0 0
    0 0 1 0
    0 0 0 1];
    T1b22e=eye(4);T1b22e(3,4)=sum(xN(1:4));
    Tcamera2marker=Ttc2end(:,:,1);
    Tbase2marker=Tbase21b*T1b22e*T2e2camera*Tcamera2marker;

    endoU=fromPsi2Curvature(endoPsi,x0N);
    endoQ=fromCurvature2Movitation(endoU,x0N);
    endoU_=fromMovitation2Curvature(endoQ,xN);
    endoPsi_=fromCurvature2Psi(endoU_,xN);
    endoPsi_(:,1:2)=endoPsi(:,1:2);

    dp=zeros(size(Ttc2end,3),1);
    dp1=zeros(size(Ttc2end,3),3);
    for i = 1:size(Ttc2end,3)
        Ttc2endi=Ttc2end(:,:,i);
        [T,~,~,~,~,~,~,~,~,~,~]=fromPsi2Config(endoPsi_(i,:),xN);
        
        Tt_c=inv(Tbase21b*T*T2e2camera)*Tbase2marker;
        dP=Tt_c(1:3,4);
        dp1(i,:)=(Ttc2endi(1:3,4)-dP(1:3))';
        dd=norm(dp1(i,:));
        dp(i)=dd;

    end
    t=sum(abs(dp1));%abs(dp1)

    f=sum(t); 


end
        
    