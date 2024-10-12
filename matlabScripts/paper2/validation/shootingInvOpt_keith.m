function [Guess,MBP,t0,t1,t2,t3,y0,y1,y2,y3,QA,Ee]=shootingInvOpt_keith(Guess,T,ksi,MBP)
    dGuess=eye(24)*1e-9;
    dGuess(7,7)=1e-3;dGuess(8,8)=1e-3;
    dGuess(19,19)=1e-3;dGuess(20,20)=1e-3;
    lambda=1e-4; % Jacobian damping
    eps=1e-5; % residue tolerance
    [Rsd,~,~,~,~,~,~,~,~,~,~,Ee]=forInvShooting_keith(Guess,T,ksi,MBP);
    
    J= zeros(18,24);
    e_err=zeros(24,1);
    g_err=zeros(24,1);
    iter = 0;
    disp(['->Now,the itertaion is ',num2str(iter),' times, and the residue is ',num2str(norm(Rsd))]);
    cter2=10;
    cter3= 1;
    max_iter = 20;
    cter0 = zeros(5,1);
    Guess_all = zeros(max_iter,25);
    dt=0.1;
    dt2=1e-5;
    while(norm(Rsd)>eps)%|| cter2 >eps)    
        for i=1:size(Guess,1) % finite differencing for Jacobian of initial guess
            [Rsd_,~,~,~,~,~,~,~,~,~,~,Ee_]=forInvShooting_keith(Guess+dGuess(:,i),T,ksi,MBP);
            J(:,i)=(Rsd_-Rsd)/norm(dGuess(:,i));
            e_err(i)=(Ee-Ee_)/norm(dGuess(:,i));
        end

        if(iter>10)
            dt=1;
            % dt2=0;
        end
        G_ = eye(24)/(J'*J+lambda*eye(24))*J';
        % E_ = eye(24)-G_*pinv(G_);
        % E_sign = sign(e_err);
%         for i = 1:size(e_err,1)
        % for i = [8:12 20:24]
        %     t = abs(e_err(i));
        %     if(t<1e-4)
        %         g_err(i)=0;
        %     else
        %         g_err(i)=1*E_sign(i)/t;
        %     end
        % end
%         Guess_last = Guess;
        Guess=Guess-(G_*(Rsd))*dt; % update guess
        [Rsd,~,~,~,~,~,~,~,~,~,~,Ee]=forInvShooting_keith(Guess,T,ksi,MBP); % number IVP of equal to number of guessed values 
        iter = iter + 1;
        disp(['->Now,the itertaion is ',num2str(iter),' times, and the residue is ',num2str(norm(Rsd))]);
        
        if(abs(cter2-Ee)<eps/10)
            cter2=0;
        else

            cter2 = Ee;
        end
        if(iter>max_iter)
            break;
        end
        % if(norm(Rsd)<eps)
        %     Guess_all(cter3,1:24)=Guess;
        %     Guess_all(cter3,25)=Ee;
        %     cter3=cter3+1;
        %     if(cter3==size(Guess_all,1)+1)
        %         break;
        %     end
        % end
    end
    % [~,tt]=min(Guess_all(:,end)); 
    % Guess = Guess_all(tt,1:24)';
    [~,MBP,t0,y0,t1,y1,t2,y2,t3,y3,QA,Ee]=forInvShooting_keith(Guess,T,ksi,MBP);

end