function [f]=getGamma_a(x)
    global Tt;
    global Cf;
    global Res;
    xN=Res;
    xN(5)=Res(5)/10;xN(6)=Res(6)/10;
    xN(7)=Res(7)/10;
    xN(8)=Res(8)*10;xN(9)=Res(9)/100;
    x0=setX0;
    
    x0N=x0;
    Ttc2end=Tt;
    endoPsi=Cf;
    se=[1 2 3   4:20 ]; % 
    Ttc2end=Ttc2end(:,:,se);
    endoPsi=endoPsi(se,:);
    
    gamma_a=x;
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
    for i = [6 7:17]%2:size(Ttc2end,3)
        Ttc2endi=Ttc2end(:,:,i);
        [T,~,~,~,~,~,~,~,~,~,~]=fromPsi2Config(endoPsi_(i,:),xN);
        
%         Tt_c=Tbase2marker*inv(Ttc2endi)*inv(T2e2camera);
        Tt_c=inv(Tcamera2marker)*Ttc2endi;
        axang1=rotm2axang(Ttc2endi(1:3,1:3));
        
        
%         dp1(i,:)=(axang1(4)-x)';
%         dd=norm(dp1(i,:));
        dp(i)=abs(axang1(4)-x+gamma_g);

    end
    t=abs(sum(dp));%abs(dp1)

    f=sum(t); 
end



