function [Guess,MBP,t0,t1,t2,t3,y0,y1,y2,y3,ksi]=shootingStiffOpt_keith(Guess,qa,T,MBP)
    dGuess=eye(32)*1e-9;
    lambda=1e-6; % Jacobian damping
    eps=1e-5; % residue tolerance
    Rsd=forStiffShooting_keith(Guess,qa,T,MBP);
    
    J= zeros(26,32);
    iter = 0;
    disp(['当前迭代次数为',num2str(iter),', 误差为',num2str(norm(Rsd))]);
    while(norm(Rsd)>eps)    
        for i=1:size(Guess,1) % finite differencing for Jacobian of initial guess
            [Rsd_]=forStiffShooting_keith(Guess+dGuess(:,i),qa,T,MBP);
            J(:,i)=(Rsd_-Rsd)/norm(dGuess(:,i));
        end
    
        Guess=Guess-eye(32)/(J'*J+lambda*eye(32))*J'*(Rsd); % update guess
        Rsd=forStiffShooting_keith(Guess,qa,T,MBP);
        iter = iter + 1;
        disp(['当前迭代次数为',num2str(iter),', 误差为',num2str(norm(Rsd))]);
    end
    [~,MBP,t0,y0,t1,y1,t2,y2,t3,y3,ksi]=forStiffShooting_keith(Guess,qa,T,MBP);

end