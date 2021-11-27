function f=costFunc(x)

    xN=x;
    xN(5)=x(5)/10;xN(6)=x(6)/10;xN(7)=x(7)/100;
    x0=setX0;
    x0N=x0;
    [Ttc2end,endoPsi]=getTandConfig;
    se=[1 2 3   4:16 18:20 ]; 
    Ttc2end=Ttc2end(:,:,se);
    endoPsi=endoPsi(se,:);
    P0=Ttc2end(1:3,4,1)';
    gamma3=xN(7);
    Tgamma3=[cos(gamma3) -sin(gamma3) 0 0
    sin(gamma3) cos(gamma3) 0 0
    0 0 1 0
    0 0 0 1];
    P1end=inv(Tgamma3)*[P0 1]';
    P1tc=P1end+[0 0 sum(xN(1:4))+endoPsi(1,2) 0]';
    dp=zeros(size(Ttc2end,3),1);
    dp1=zeros(size(Ttc2end,3),3);

    for i = 1:size(endoPsi,1)
        endoQ=calcQFromPsi(endoPsi(i,:),x0N);
        endoPsi_=calcPsiFromQ(endoQ,xN);
        [T,~,~,~,~,~,~,~,~,~,~]=fromPsi2Config(endoPsi_,xN);
        dP=inv(T*Tgamma3)*P1tc;
        Ttc2endi=Ttc2end(:,:,i);
        dp1(i,:)=(Ttc2endi(1:3,4)-dP(1:3))';
        dd=norm(dp1(i,:));
        dp(i)=dd;
%         endoPsi(i,:)-endoPsi_
    end
        dp(1)=[];
    
        t=norm(sum(dp1));
        t2=sum(dp)-max(dp);
        t3=t2/(size(dp,1)-1);
        f=t3;
%     f=sum(dp);
end