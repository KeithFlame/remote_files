function [Guess,MBP,t0,t1,t2,t3,y0,y1,y2,y3,QA,Ee]=shootingInvOpt_keith(Guess,T,ksi,MBP)
    dGuess=eye(24)*1e-9;
    dGuess(7,7)=1e-3;dGuess(8,8)=1e-3;
    dGuess(19,19)=1e-3;dGuess(20,20)=1e-3;
    lambda=1e-6; % Jacobian damping
    eps=1e-4; % residue tolerance
    [Rsd,~,~,~,~,~,~,~,~,~,~,Ee]=forInvShooting_keith(Guess,T,ksi,MBP);
    
    J= zeros(18,24);
    e_err = zeros(24,1);
    g_err = zeros(24,1);
    cter2 = 10;
    cter3 = 1;
    Guess_all = zeros(30,25);
    iter = 0;
%     disp(['->Now,the inverse kinematics itertaion is ',num2str(iter),' times, which residue is ',num2str(norm(Rsd))]);
    while(norm(Rsd)>eps || cter2>eps)    
        for i=1:size(Guess,1) % finite differencing for Jacobian of initial guess
            [Rsd_,~,~,~,~,~,~,~,~,~,~,Ee_]=forInvShooting_keith(Guess+dGuess(:,i),T,ksi,MBP);
            J(:,i)=(Rsd_-Rsd)/norm(dGuess(:,i));
            e_err(i)=(Ee-Ee_)/norm(dGuess(:,i));
        end
        dt=0.1;
        if(iter>10)
            dt=1;
        end
        if(norm(Rsd)>1e-4)
            dt2 = 1e-5;
        else
            dt2 = 0.001;
        end

        Guess_tem = Guess;
        G_ = eye(24)/(J'*J+lambda*eye(24))*J';
        E_ = eye(24,24)-G_*pinv(G_);
        E_sign = sign(e_err);
        for i = [8:12 20:24]
            t = abs(e_err(i));
            if(t<1e-4)
                g_err(i)=0;
            else
                g_err(i)=E_sign(i)/t;
            end
        end
        Guess_last = Guess;
        
%         Guess=Guess-eye(24)/(J'*J+lambda*eye(24))*J'*(Rsd)*dt; % update guess
        Guess=Guess-(G_*(Rsd)-dt2*E_*g_err)*dt; % update guess
        [Rsd,~,~,~,~,~,~,~,~,~,QA,Ee]=forInvShooting_keith(Guess,T,ksi,MBP); % number IVP of equal to number of guessed values
        if(max(abs(QA([3:6 9:12])))>10e-3)
            dt = find(abs(QA([3:6 9:12]))>10e-3);
            Guess0=Guess([9:12 21:24]);
            Guess1=Guess_tem([9:12 21:24]);
            Guess0(dt)=Guess1(dt);
            Guess([9:12 21:24])=Guess0;
            [Rsd,~,~,~,~,~,~,~,~,~,~,Ee]=forInvShooting_keith(Guess,T,ksi,MBP); 
        end
        iter = iter + 1;
%         disp(['->Now,the inverse kinematics itertaion is ',num2str(iter),' times, which residue is ',num2str(norm(Rsd))]);
    
        if(abs(cter2-Ee)<eps)
            cter2 = 0;
        else
            cter2 = Ee;
        end
        if(norm(Rsd)<eps)
            Guess_all(cter3,1:24)=Guess';
            Guess_all(cter3,25)=Ee;
            cter3=cter3+1;
            if(cter3==size(Guess_all,1)+1)
                break;
            end
        end
        if(iter>80)
            break;
        end
    end
    Guess_all(all(Guess_all==0,2),:)=[];
    if(isempty(Guess_all))
        disp("逆运动学求解错误！！！！");
        return;
    end
    [~,tt]=min(Guess_all(:,end));
    Guess = Guess_all(tt,1:24)';
    [~,MBP,t0,y0,t1,y1,t2,y2,t3,y3,QA,Ee]=forInvShooting_keith(Guess,T,ksi,MBP);

end