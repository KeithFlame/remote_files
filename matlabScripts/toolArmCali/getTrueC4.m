function f=getTrueC4(x)
    global LS;
    global precisionTarget;
    global iter;
    xN=LS;
    xN(9)=0;
    xN(4)=15;
    x(2)=x(2)*100;
    if(x(2)<xN(1))
        x(2)=xN(1);
    end
    dttc=LS(10);
    [T,~,~,~,~,~,~,~,~,~,~]=fromPsi2Config(x,xN);
    RP=precisionTarget(iter,:);
    P=RP(1:3)'-[0 0 dttc]';
    R=eye(3);
    axang=rotm2axang(T(1:3,1:3)'*R);
    dAngle=abs(axang(4)*180/pi);
    dP=norm(P-T(1:3,4));
    f=dAngle+dP;
end
