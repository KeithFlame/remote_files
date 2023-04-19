function [Guess,t0,t1,t2,t3,y0,y1,y2,y3]=shootingOpt_keith(Guess,qa,ksi)
    dGuess=eye(20)*1e-9;
%     dGuess(10,10) = 1;dGuess(11,11) = 1;dGuess(12,12) = 1;
    lambda=1e-6; % Jacobian damping
    eps=2e-5; % residue tolerance
    
%     nc=[0 0 0]';mc=[0 0 0]';v=[0 0 0 0]';Guess=[nc;mc;v;nc;mc;v]; % guessed base force moment strain
    [Rsd,t0,y0,t1,y1,t2,y2,t3,y3]=forShooting_keith(Guess,qa,ksi);
    
    J= zeros(20,20);
    iter = int16(0);
    info = sprintf('-> Now the iteration is %04d. and error is %06f',iter,norm(Rsd));
    disp(info);
    while(norm(Rsd)>eps)    
        for i=1:20 % finite differencing for Jacobian of initial guess
            [Rsd_]=forShooting_keith(Guess+dGuess(:,i),qa,ksi);
            J(:,i)=(Rsd_-Rsd)/norm(dGuess(:,i));
        end
    
        Guess=Guess-eye(20)/(J'*J+lambda*eye(20))*J'*(Rsd); % update guess
        [Rsd,t0,y0,t1,y1,t2,y2,t3,y3]=forShooting_keith(Guess,qa,ksi); % number IVP of equal to number of guessed values 
        iter = iter + 1;
        info = sprintf('-> Now the iteration is %04d. and error is %06f',iter,norm(Rsd));
        disp(info);
    end

end