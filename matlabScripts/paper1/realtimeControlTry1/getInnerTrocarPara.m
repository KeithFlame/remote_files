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
    
    if abs(e)>tol1 %这种情况下是用二次插值法
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
            %采取黄金分割法得到更大的两段
            if x>=xm %从a，x，b区间段中 找最大的一段，乘以GOLD作为下一次的步长
                e = a-x;
            else
                e = b-x;
            end
            d = GOLD*e;%二次插值法不被接受，采用golden section
        else
            d = p/q;
            u = x + d; %采用二次插值法
            if u-a<tol2||b-u<tol2 %如果u值很接近a或者b，将步长设为xm和x的长度
                d = MySignFunc(tol1,xm-x);
            end
        end
        
    else %采用黄金分割
        if x>=xm
            e = a-x;
        else
            e = b-x;
        end
        d = GOLD*e;%二次插值法不被接受，采用golden section
    end
    
    if abs(d) >= tol1 %步长选择tol1和d中较小的一个
        u = x+d;
    else
        u = x+MySignFunc(tol1,d);
    end
    
    fu = func(u);
    if fu<=fx %如果fu小于fx,说明下次迭代为区间不是在[a,x]，就是在[x,b]
        if u>=x %如果u>=x，则在[x,b]，反之在[a,x]
            a = x;
        else
            b = x;
        end
        v = w;w = x;x = u;
        fv = fw;fw = fx;fx = fu;
    else%反之，说明下次迭代区间不是[a,u],就是[u,b]
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
