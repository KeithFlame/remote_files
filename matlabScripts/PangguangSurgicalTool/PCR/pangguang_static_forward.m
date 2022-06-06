% for pangguang parallel continuum robot forward kinetostatic
%
% author Keith W.
% Ver. 1.0
% Date 05.15.2022

%% 已知条件
SP = parallel_structure;
% R_target = eul2rotm([0 0 0]);
% P_target = [0 0 40] * 1e-3;
% target = [R_target P_target; [0 0 0 1]];

qa=[20.0 ...
    20.0 ...
    20.0 ] * 1e-3; % PHI D 
Fe=[0.0 0.0 0.0]';Me=[0 0 0]';
fe=[0 0 0]';le=[0 0 0]';

%% 计算处理
[Guess,t1,t2,y1,y2]=shootingMethod(qa,Fe,Me,fe,le,SP);

%%  函数
%% ===Model Functions=== %%
function [Guess,t1,t2,y1,y2]=shootingMethod(qa,Fe,Me,fe,le,SP)
    dGuess=eye(10)*1e-5;    
    lambda=5e-10; % Jacobian damping
    eps=1e-5; % residue tolerance

    nc=[0 0 0]';mc=[0 0 0]';v=0;Guess1=[nc;mc;v]; 
    rod_number = size(SP.ps,3);
    Guess1_length = size(Guess1, 1);
    Guess = zeros(rod_number * Guess1_length, 1);
    for i = 1 : rod_number
        Guess((1 + (i - 1) *Guess1_length):(i * Guess1_length)) = Guess1;
    end
    % guessed base force moment strain
    [Rsd,t1,y1]=forShooting(Guess,qa,Fe,Me,fe,le,SP);

    tic
    while(norm(Rsd)>eps)
        disp(['residue0 = ' num2str(norm(Rsd))]);
    
        for i=1:10 % finite differencing for Jacobian of initial guess
            [Rsd_]=forShooting(Guess+dGuess(:,i),qa,Fe,Me,fe,le,SP);
            J(:,i)=(Rsd_-Rsd)/norm(dGuess(:,i));
        end
    
        Guess=Guess-1/(J'*J+lambda*eye(10))*J'*(Rsd); % update guess
        [Rsd,t1,y1,t2,y2]=forShooting(Guess,qa,Fe,Me,fe,le,SP); % number IVP of equal to number of guessed values
         toc   
% % % % %         if(plotFlag==1)
% % % % %             pause(0.02);
% % % % %             delete(hShape);
% % % % %     	    size1=size(y1(:,1:3));size2=size(y2(:,1:3));
% % % % %             n1=size1(1);n2=size2(1);
% % % % %             [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1,n2],SP);
% % % % %         end
    end
end

function [Rsd,t1,yL]=forShooting(Guess,qa,Fe,Me,fe,le,SP)
    % do an IVP shooting
    
    unknow_number_each_rod = 7;
    rod_number = size(Guess,1)/unknow_number_each_rod;
    yL = zeros(13,rod_number);
    PS = SP.ps;
    for i = 1:rod_number
        nc=Guess((1 + (i-1)*unknow_number_each_rod):(3 + (i-1)*unknow_number_each_rod)); % base internal force
        mc=Guess((4 + (i-1)*unknow_number_each_rod):(6 + (i-1)*unknow_number_each_rod)); % base internal load
        v=[0 0; 0 0; qa(i) Guess(i * unknow_number_each_rod)]; % rod elongation strain
        
        y0=[PS(:,i)' nc' mc' 0]';
        [t1,y1]=odeCosserat1(y0,v,SP,fe,le,1e-3); % first seg y=p,R,n,m,q for each row
        yL1=y1(end,:)';
        
        qe=SP.sp.L * v(3,1); % elongation
        yL(:,i)=[yL1(1:13);
%             yL1(10:12);%+2*(cross(R1*SP.r11,R1*SP.Ke1*v(:,1))+cross(R1*SP.r12,R1*SP.Ke1*v(:,2)));
            qe]; % ? start of 2nd seg
                
    end

    Rsd=zeros(Guess,1);

    DS = SP.ds;
    Rsd(1) = yL(end - 1, 1) - (qa(1) + yL(end, 1));
    for i = 2:rod_number
        % actuation
        %target pose error
        ds_t = DS(:, 1) - DS(:, i);

        %calc pose error
        ds_cp = yL(1:3, 1)-eul2rotm(yL(4:6,1))*yL(1:3, i);
        aa1 = [yL(4:6, 1)' / norm(yL(4:6, 1)) norm(yL(4:6, 1))];
        aai = [yL(4:6, i)' / norm(yL(4:6, i)) norm(yL(4:6, i))];
        ds_co = rotm2axang(axang2rotm(aa1)\axang2rotm(aai));
        ds_co = ds_co(1:3)' * ds_co(4);
        ds_c = [ds_cp;ds_co];

        Rsd((2 + (i-2)*unknow_number_each_rod):(7 + (i-2)*unknow_number_each_rod)) ...
            = ds_c - ds_t;
        Rsd(8 + (i - 2) * unknow_number_each_rod) = yL(end - 1, i) - (qa(i) + yL(end, i));
    end
                
    Rsd((unknow_number_each_rod*rod_number+1):(unknow_number_each_rod*rod_number+3))...
        =Fe-sum(yL(:,7:9),2); % boundary conditions
    Rsd((unknow_number_each_rod*rod_number+4):(unknow_number_each_rod*rod_number+6))...
        =Me-sum(yL(:,10:12),2);
end

function [t,y,U]=odeCosserat1(y0,v,SP,fe,le,step)
%-----Integral of IVP using difference equation-------
% y0- initial condition
% v- rod elongation strain
% SP- structure properties
% fe, le- external distributed loads
% step- integrating step size
%----------info-----------%
% ver f1.0
% by Yuyang Chen
% date 20200524
%-------------------------------------------------------------------------%
    DoF=length(y0);
    Kb=SP.sp.Kb;

    N=round(v(end,1)/step)+1;
    t=linspace(0,v(end,1),N)';
    step=v(end,1)/(N-1);

    y=zeros(N,DoF);U=zeros(N,3);
    y(1,:)=y0';
%     R_ = [y0(4:6)' / norm(y0(4:6)) norm(y0(4:6))];
%     R = axang2rotm(R_);
% %     R=[y0(4:6) y0(7:9) y0(10:12)];
%     m=y0(10:12);
    for i=1:N-1
        p=y(i,1:3)';
        R_ = [y(i,4:6)' / norm(y(i,4:6)) norm(y(i,4:6))];
        R = axang2rotm(R_);        
%         R=[y(i,4:6);y(i,7:9);y(i,10:12)]';
        n=y(i,7:9)';
        m=y(i,10:12)';
        q=y(i,13)';
        u=Kb\R'*m;u(3)=0;
        
        %constant-curvature-based evolution
        theta=step*norm(u);delta=-atan2(u(2),u(1))+pi/2;
        costheta=cos(theta);sintheta=sin(theta);cosdelta=cos(delta);sindelta=sin(delta); % cache
        if(theta~=0)
            p_dot=R*( step/theta*[cosdelta*(1-costheta) sindelta*(costheta-1) sintheta]' );
            R_dot=[cosdelta^2*costheta+sindelta^2 -sindelta*cosdelta*(costheta-1) cosdelta*sintheta;...
                   sindelta*cosdelta*(1-costheta) cosdelta^2+costheta*sindelta^2 -sindelta*sintheta;...
                  -cosdelta*sintheta sindelta*sintheta costheta];
        else
            p_dot=R*[0 0 step]';
            R_dot=eye(3);
        end
        p=p+p_dot;
        R=R*R_dot;
    
        n_dot=-fe*step;
        m_dot=-S(p_dot)*n - le*step;% -...
%         2*R*(( cross(S(u)*SP.r11,Ke1*v(:,1))+cross(SP.r11,S(u)*Ke1*v(:,1)) + ...
%                cross(S(u)*SP.r12,Ke1*v(:,2))+cross(SP.r12,S(u)*Ke1*v(:,2)) )*mod_SegIdx + ...
%              ( cross(S(u)*SP.r21,Ke2*v(:,3))+cross(SP.r21,S(u)*Ke2*v(:,3)) + ...
%                cross(S(u)*SP.r22,Ke2*v(:,4))+cross(SP.r22,S(u)*Ke2*v(:,4)) ))*step;
        n=n+n_dot;
        m=m+m_dot;
        
        q_dot=norm(u*step)*0;
        q=q+q_dot;
        y(i+1,:)=[p;R(:,1);R(:,2);R(:,3);n;m;q]';
        U(i,:)=u';
    end
    U(end,:)=Kb\R'*m;
end


% function [R]=Expm(u)
% %simplified calculation for exponetial map (u in R3)
% theta=norm(u);
% if(theta == 0)
%     R=eye(3);
% else
%     un=u/theta;
%     R=cos(theta)*eye(3)+(1-cos(theta))*(un*un')+sin(theta)*S(un);
% end
% end
% 
% function T = S( p )
% %this function gives the skew-symmetric matrix of the vector p
% 
% if(length(p)==3)
%     T=[0 -p(3) p(2); p(3) 0 -p(1);-p(2) p(1) 0];
% elseif(length(p)==6)
%     R=[0 -p(6) p(5); p(6) 0 -p(4);-p(5) p(4) 0];
%     T=[R p(1:3);zeros(1,4)];
% end
% end

