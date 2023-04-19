function [Guess,MBP,t0,t1,t2,t3,y0,y1,y2,y3]=shootingFkOpt_keith(Guess,qa,ksi,MBP)
    dGuess=eye(20)*1e-9;
    lambda=1e-6; % Jacobian damping
    eps=2e-5; % residue tolerance
    Rsd=forFkShooting_keith(Guess,qa,ksi,MBP);
    
    J= zeros(20,20);
    iter = 0;
    disp(['当前迭代次数为',num2str(iter),', 误差为',num2str(norm(Rsd))]);
    while(norm(Rsd)>eps)    
        for i=1:size(Guess,1) % finite differencing for Jacobian of initial guess
            [Rsd_]=forFkShooting_keith(Guess+dGuess(:,i),qa,ksi,MBP);
            J(:,i)=(Rsd_-Rsd)/norm(dGuess(:,i));
        end
    
        Guess=Guess-eye(20)/(J'*J+lambda*eye(20))*J'*(Rsd); % update guess
        Rsd=forFkShooting_keith(Guess,qa,ksi,MBP); % number IVP of equal to number of guessed values 
        iter = iter + 1;
        disp(['当前迭代次数为',num2str(iter),', 误差为',num2str(norm(Rsd))]);
    end
    [~,MBP,t0,y0,t1,y1,t2,y2,t3,y3]=forFkShooting_keith(Guess,qa,ksi,MBP);

end