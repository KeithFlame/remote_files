function [Guess,MBP,t0,t1,t2,t3,y0,y1,y2,y3,ksi]=shootingStiffOpt_keith(Guess,qa,T,MBP)
    dGuess=eye(32)*1e-9;
    lambda=1e-6; % Jacobian damping
    eps=4e-5; % residue tolerance
    Rsd=forStiffShooting_keith(Guess,qa,T,MBP);
    
    J= zeros(26,32);
    iter = 0;
    disp(['->Now,the stiffness kinematics itertaion is ',num2str(iter),' times, which residue is ',num2str(norm(Rsd))]);
    num = 100;
    Guess_block=zeros(num+1,33);
    Guess_block(1,1:32)=Guess';
    Guess_block(1,33)=norm(Rsd);
    dt=0.1;
    while(norm(Rsd)>eps)
        for i=1:size(Guess,1) % finite differencing for Jacobian of initial guess
            [Rsd_]=forStiffShooting_keith(Guess+dGuess(:,i),qa,T,MBP);
            J(:,i)=(Rsd_-Rsd)/norm(dGuess(:,i));
        end
        if(iter>10)
            dt=0.2;
        end
        Guess=Guess-eye(32)/(J'*J+lambda*eye(32))*J'*(Rsd)*dt; % update guess
        for i = [22:24 27:29 31:32]
            if(Guess(i)<-0.7)
                Guess(i)=-0.7;
            end
        end
        Rsd=forStiffShooting_keith(Guess,qa,T,MBP);
        iter = iter + 1;
        Guess_block(iter+1,1:32)=Guess';
        Guess_block(iter+1,33)=norm(Rsd);
        disp(['->Now,the stiffness kinematics itertaion is ',num2str(iter),' times, which residue is ',num2str(norm(Rsd))]);
        if(iter>num)
            break;
        end
    end
    cter = Guess_block(:,33);
    [~,b]=min(cter);
    Guess=Guess_block(b,1:32)';
    [Rsd,MBP,t0,y0,t1,y1,t2,y2,t3,y3,ksi]=forStiffShooting_keith(Guess,qa,T,MBP);
    disp(['->Now,the itertaion is ',num2str(iter),' times, and the residue is ',num2str(norm(Rsd))]);

end