function [qa, S] = inverse_snake(T,MBP, FM,discrete_element)


if (nargin == 2)
    FM = zeros(12,1);
    MBP.discrete_element = 1e-3;
end

if(nargin == 3)
    MBP.discrete_element = 1e-3;
end
if(nargin == 4)
    MBP.discrete_element = discrete_element*1e-3;
end
Fe=FM(1:3);Me=FM(4:6);
fe=FM(7:9);le=FM(10:12);%distributed not active in this version

pose = zeros(6,1);
pose(1:3) = T(1:3,4);
axang = rotmaxang(T(1:3,1:3));
pose(4:6) = axang(1:3)'*axang(4);

%% execute
[QA,Guess,Res,t1,t2,y1,y2]=shootingOpt2_SingleIK(T,Fe,Me,fe,le,MBP);
p=y2(end,1:3)';R=[y2(end,4:6);y2(end,7:9);y2(end,10:12)]';
Tend=[R p*1000;0 0 0 1];
u=calcCurvature(y1,y2,t1,t2,MBP);
t0 = round(u(1,:)*1000) + u(2,:)/1000;
p0 = [y1(:,1:3);y2(:,1:3)];
S = [p0*1000 t0'];
qa = QA;
end
function [QA,Guess,Res,t1,t2,y1,y2]=shootingOpt2_SingleIK(T,Fe,Me,fe,le,MBP)
    
    lambda=5e-2;
    eps=1e-4;
    Guess = zeros(5,1);
    N_ukn=length(Guess);
    dGuess=eye(N_ukn)*1e-6;
    count = 0;
    
    [Rsd,t1,y1,t2,y2,QA]=invShooting(Guess,Tg,Fe,Me,fe,le,MP);
    while(norm(Rsd)>eps)
    
        for i=1:N_ukn%Jacobian(DoF_rsd,DoF_guess)
            [Rsd_]=invShooting3(Guess+dGuess(:,i),Tg,Fe,Me,fe,le,MP);
            J(:,i)=(Rsd_-Rsd)/norm(dGuess(:,i));
        end
    
        Guess=Guess-J'/(J'*J+lambda*eye(N_ukn))*(Rsd);
        if(Guess<0)
            Guess = 0;
        end
        [Rsd,t1,y1,t2,y2,QA]=invShooting(Guess,Tg,Fe,Me,fe,le,MP);
        
        count=count+1;
    
        if(count>20)
            break;
        end
    end

end

%---5DoF reverted integrating, considering stem bending
function [Rsd,t1,y1,t2,y2,QA]=invShooting3(Guess,Tg,Fe,Me,fe,le,MP)

Rg=Tg(1:3,1:3);pg=Tg(1:3,4);
v=zeros(3,4);v(end,:)=Guess(1:4)';
MP.Lso=Guess(5);
nc=Fe;%assume fe=0.
mc=Me-2*( cross(Rg*MP.r21,Rg*MP.Ke2*v(:,3)) + cross(Rg*MP.r22,Rg*MP.Ke2*v(:,4)) );
step=1e-3;

yL2=[pg' Rg(:,1)' Rg(:,2)' Rg(:,3)' nc' mc' 0 0]';
yL2_=[pg'-MP.Lg*[0 0 1]*Rg' Rg(:,1)' Rg(:,2)' Rg(:,3)' nc' mc' 0 0]';
[t2,y2]=odeCosserat2(yL2_,v,2,MP,fe,le,step);
y2=[yL2';y2];
t2=[MP.Lg+MP.L2+MP.Lr+MP.L1+MP.ell;t2];
R1=[y2(end,4:6);y2(end,7:9);y2(end,10:12)]';

yL1=[y2(end,1:18)';0;0;y2(end,19:20)'];
yL1_=[yL1(1:3)'-MP.Lr*[0 0 1]*R1' yL1(4:15)' (yL1(16:18)-2*(cross(R1*MP.r11,R1*MP.Ke1*v(:,1))+cross(R1*MP.r12,R1*MP.Ke1*v(:,2))) )' yL1(19:22)'];

[t1,y1]=odeCosserat2(yL1_,v,1,MP,fe,le,step);
y1=[yL1';y1];
t1=[MP.Lr+MP.L1+MP.ell;t1];
yL0=y1(end,:)';
%R0=[y1(end,4:6);y1(end,7:9);y1(end,10:12)]';
%p0=y1(end,1:3)';

[t0,y0]=odeCosserat2(yL0,v,0,MP,fe,le,step);
y0=[yL0';y0];
t0=[MP.ell;t0];
yL_stem=y0(end,:)';
R0=[y0(end,4:6);y0(end,7:9);y0(end,10:12)]';
p0=y0(end,1:3)';

y1=[y1;y0];t1=[t1;t0];
qe1=(MP.L+MP.L1)*[v(3,1) v(3,2)]';
qe2=(MP.L+MP.L1+MP.L2+MP.Lr)*[v(3,3) v(3,4)]';%not used currently
QA=[[0 0 1]*Logm(R0);MP.ell;y0(end,19:20)'-qe1;y0(end,21:22)'-qe2];

Rsd=zeros(5,1);

Rsd(1:3)=p0;
Rsd(4:5)=[1 0 0;0 1 0]*Logm(R0);
t1=fliplr(t1')';t2=fliplr(t2')';
y1=fliplr(y1')';y2=fliplr(y2')';

end

