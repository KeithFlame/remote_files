function [Guess,MBP,t0,t1,t2,t3,y0,y1,y2,y3,QA]=shootingInvOpt2_keith(Guess,T,ksi,MBP)
    dGuess=eye(24)*1e-9;
    dGuess(7,7)=1e-3;dGuess(8,8)=1e-3;
    dGuess(19,19)=1e-3;dGuess(20,20)=1e-3;
    lambda=1e-6; % Jacobian damping
    eps=1e-6; % residue tolerance
    Rsd=forInvShooting2_keith(Guess,T,ksi,MBP);
    
    J= zeros(24,24);
    iter = 0;
    disp(['->Now,the itertaion is ',num2str(iter),' times, and the residue is ',num2str(norm(Rsd))]);
    while(norm(Rsd)>eps)    
        for i=1:size(Guess,1) % finite differencing for Jacobian of initial guess
            Rsd_=forInvShooting2_keith(Guess+dGuess(:,i),T,ksi,MBP);
            J(:,i)=(Rsd_-Rsd)/norm(dGuess(:,i));
        end
        dt=0.1;
        if(iter>10)
            dt=1;
        end
        Guess=Guess-eye(24)/(J'*J+lambda*eye(24))*J'*(Rsd)*dt; % update guess
        Rsd=forInvShooting2_keith(Guess,T,ksi,MBP); % number IVP of equal to number of guessed values 
        iter = iter + 1;
        disp(['->Now,the itertaion is ',num2str(iter),' times, and the residue is ',num2str(norm(Rsd))]);
    end
    [~,MBP,t0,y0,t1,y1,t2,y2,t3,y3,QA]=forInvShooting2_keith(Guess,T,ksi,MBP);

end