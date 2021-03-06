function [config, flag] = getInnerTrocarPara(psi)
%
%
%
    cter=getCurvatureNoClearance(psi);

    tro=getStructurePara;
    LS=tro.LS;

        if(cter==1)
            flag=0;
            if(psi(2)>LS(1))
                lso=psi(2)-LS(1);
                l1o=LS(1);
            else
                lso=0;
                l1o=psi(2);
            end
            config=[0 0 0 0 0 0 lso l1o 0 psi(4)];
            return;
        end

    if(psi(2)>sum(LS(1:3))*0.99)
        config=getBaseStemInTrocar;
        flag=1;
    else
        config=getSeg1InTrocar;
        if(sum(config([2 4]))>LS(1))
            flag=2;
            config(1)=(sum(config([2 4]))-LS(1))/tro.zeta;
            config(2)=LS(1)-config(4);
            config(9)=config(1)+config(2);
        else
            flag=3;
        end
        
    end

end




function config=getBaseStemInTrocar()

    f=@getThetasi;
    thetasi=BrentMethod(f,0,0.03,0.06);
    sTsi=sin(thetasi);
    cTsi=cos(thetasi);
    trocar = getStructurePara;
    c=trocar.c;
    zeta=trocar.zeta;
    LS=trocar.LS;
    L1=LS(1);

    KP=setPsi_U;
    psi=KP.psi;
    theta1=psi(3);

    l=psi(2);
    d=c*sTsi./(1-cTsi);
    lsi=d.*thetasi./sTsi;
    lso=l-sum(LS(1:3))+d-lsi;
    thetaso=(theta1-thetasi).*lso*zeta./(lso*zeta+L1);
    theta1o=theta1-thetaso-thetasi;
    l1i=0;theta1i=0;l1o=L1;
    config=[lsi l1i lso l1o thetasi theta1i thetaso theta1o d psi(4)];
end

function config=getSeg1InTrocar()

    f=@getTheta1i;
    theta1i=BrentMethod(f,0,0.2,0.4);
    sT1i=sin(theta1i);
    cT1i=cos(theta1i);
    trocar = getStructurePara;
    c=trocar.c;
    LS=trocar.LS;

    KP=setPsi_U;
    psi=KP.psi;
    theta1=psi(3);

    l=psi(2);
    d=c*sT1i/(1-cT1i);
    l1i=d*theta1i/sT1i;
    l1o=l-sum(LS(2:3))+d-l1i;
    thetaso=0;
    theta1o=theta1-thetaso-theta1i;
    thetasi=0;lso=0;lsi=0;
    config=[lsi l1i lso l1o thetasi theta1i thetaso theta1o d psi(4)];
end

function x_min = BrentMethod(func,a,b,c,tol,iterNum,plotFlag)
%Brent's Method fminbnd

if nargin < 7
    plotFlag = 0;
end

if nargin < 6
    iterNum = 100;
end

if nargin < 5
    tol = 10^(-8);
end

if (a-b)*(b-c)<=0
    x_min = 0;
    disp('a<b<c or a>b>c is needed!');
    return ;
end

if (a>c)
    b = a;
    a = c;
else
    b = c;
end

if plotFlag == 1
    figure();
    abInt = (b-a)/100;
    plot(a:abInt:b,func(a:abInt:b));
    hold on;
end

GOLD = 1 - (sqrt(5)-1)/2.0;

x = b;
w = b;
v = b;

fw = func(w);
fx = fw;
fv = fw;

e = 0;
d = 0;
for i=1:1:iterNum
    xm = (a+b)/2.0;
    tol1 = tol*abs(x)+eps;
    tol2 = 2*tol1;
    if plotFlag == 1
        pause(0.15);
        plot(x,fx,'-ro');
    end
    
    if abs(x-xm) <= (tol2-0.5*(b-a))
        x_min = x;
        return;
    end
    
    if abs(e)>tol1 %????????????????????????????????????
        r = (x-w)*(fx-fv);
        q = (x-v)*(fx-fw);
        p = (x-v)*q - (x-w)*r;
        q = 2.0*(q-r);
        if q>0
            p = -p;
        end
        
        q = abs(q);
        etemp = e;
        e = d;
        
        if abs(p) >= abs(0.5*q*etemp)||p<=q*(a-x)||p>=q*(b-x)
            %??????????????????????????????????????????
            if x>=xm %???a???x???b???????????? ???????????????????????????GOLD????????????????????????
                e = a-x;
            else
                e = b-x;
            end
            d = GOLD*e;%????????????????????????????????????golden section
        else
            d = p/q;
            u = x + d; %?????????????????????
            if u-a<tol2||b-u<tol2 %??????u????????????a??????b??????????????????xm???x?????????
                d = MySignFunc(tol1,xm-x);
            end
        end
        
    else %??????????????????
        if x>=xm
            e = a-x;
        else
            e = b-x;
        end
        d = GOLD*e;%????????????????????????????????????golden section
    end
    
    if abs(d) >= tol1 %????????????tol1???d??????????????????
        u = x+d;
    else
        u = x+MySignFunc(tol1,d);
    end
    
    fu = func(u);
    if fu<=fx %??????fu??????fx,????????????????????????????????????[a,x]????????????[x,b]
        if u>=x %??????u>=x?????????[x,b]????????????[a,x]
            a = x;
        else
            b = x;
        end
        v = w;w = x;x = u;
        fv = fw;fw = fx;fx = fu;
    else%???????????????????????????????????????[a,u],??????[u,b]
        if u<x
            a = u;
        else
            b = u;
        end
        
        if fu<=fw || w==x
            v = w; w = u;
            fv = fw; fw = fu;
        else 
            if fu<=fv||v==x||v==w
                v=u;
                fv=fu;
            end
        end
    end
end
end

function a = MySignFunc(a,x)
a = abs(a);
if x<0
    a = -a;
end
end

function f=getThetasi(thetasi)
    trocar = getStructurePara;
    c=trocar.c;
    LS=trocar.LS;
    zeta=trocar.zeta;
    Lstem=trocar.Lstem;
    L1=LS(1);
    k1=trocar.K1;
    ks=k1/zeta;

    KP=setPsi_U;
    psi=KP.psi;
    u=KP.u;
    us=u(1);u1=u(2);
    l=psi(2);
    theta1=psi(3);
    
    sTsi=sin(thetasi);
    cTsi=cos(thetasi);
    
    d=c*sTsi./(1-cTsi);
    lsi=d.*thetasi./sTsi;
    lso=l-sum(LS(1:3))+d-lsi;
    thetaso=(theta1-thetasi).*lso*zeta./(lso*zeta+L1);
    theta1o=theta1-thetaso-thetasi;
    f=0.5*k1*L1*(theta1o/L1-u1)^2+0.5*ks*(lsi*(thetasi/lsi-us)^2+lso*(thetaso/lso-us)^2+(Lstem-lsi-lso)*us^2);
end

function f=getTheta1i(theta1i)
    sT1i=sin(theta1i);
    cT1i=cos(theta1i);
    trocar = getStructurePara;
    c=trocar.c;
    LS=trocar.LS;
    L1=LS(1);
    Lstem=trocar.Lstem;
    zeta=trocar.zeta;
    k1=trocar.K1;
    ks=k1/zeta;

    KP=setPsi_U;
    psi=KP.psi;
    u=KP.u;
    us=u(1);u1=u(2);
    theta1=psi(3);

    l=psi(2);
    d=c*sT1i/(1-cT1i);
    l1i=d.*theta1i/sT1i;
    l1o=l-sum(LS(2:3))+d-l1i;
    thetaso=0;
    theta1o=theta1-thetaso-theta1i;
    if(L1-l1i-l1o)>0
        f=0.5*k1*(l1o*(theta1o/l1o-u1)^2+l1i*(theta1i/l1i-u1)^2)+0.5*k1*((L1-l1i-l1o)*u1^2)+0.5*ks*us^2*Lstem;
    else
        ls=-(L1-l1i-l1o)/zeta;
        f=0.5*k1*(l1o*(theta1o/l1o-u1)^2+(L1-l1o)*(theta1i/l1i-u1)^2)+0.5*ks*us^2*(Lstem-ls)+0.5*ks*(us-zeta*theta1i/l1i)^2*ls;
    end
end
