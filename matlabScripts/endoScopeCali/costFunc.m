function f=costFunc(x)

    xN=x;
    xN(5)=x(5)/10;xN(6)=x(6)/10;
    xN(7)=x(7)*1e10/3;
    xN(8)=x(8)*10;xN(9)=x(9)/100;
    x0=setX0;
    
    x0N=x0;
    [Ttc2end,endoPsi]=getTandConfig;
    se=[1 2 3   4:20 ]; % 
    Ttc2end=Ttc2end(:,:,se);
    endoPsi=endoPsi(se,:);
    
    
    P0=Ttc2end(1:3,4,1)';
    gamma3=xN(9);
    Tgamma3=[cos(gamma3) -sin(gamma3) 0 0
    sin(gamma3) cos(gamma3) 0 0
    0 0 1 0
    0 0 0 1];
    P1end=Tgamma3*[P0 1]';
    P1tc=P1end+[0 0 sum(xN(1:4))+endoPsi(1,2) 0]';
    endoU=fromPsi2Curvature(endoPsi,x0N);
    endoQ=fromCurvature2Movitation(endoU,x0N);
    endoU_=fromMovitation2Curvature(endoQ,xN);
    endoPsi_=fromCurvature2Psi(endoU_,xN);
    endoPsi_(:,1:2)=endoPsi(:,1:2);
%     err_P=0;
%     err_R=0;
    dp=zeros(size(Ttc2end,3),1);
    dp1=zeros(size(Ttc2end,3),3);
    for i = 1:size(Ttc2end,3)
        Ttc2endi=Ttc2end(:,:,i);
        [T,~,~,~,~,~,~,~,~,~,~]=fromPsi2Config(endoPsi_(i,:),xN);
        dP=inv(T*Tgamma3)*P1tc;
        dp1(i,:)=(Ttc2endi(1:3,4)-dP(1:3))';
        dd=norm(dp1(i,:));
        dp(i)=dd;
    end
    t=sum(abs(dp1));
%     b=sort(dp);
%     d=floor(size(b,1)/10);
%     f=sum(b(1:end-d));

%     f=max(err_P,err_R);
    f=sum(t); %(1)+t(2)
end
        
    