% for pangguang parallel continuum robot forward kinetostatic
%
% author Keith W.
% Ver. 1.0
% Date 06.09.2022

%% 已知条件
SP = parallel_structure;
qa=[20.0 ...
    20.0 ...
    20.0 ] * 1e-3; % PHI D 
Fe=[0.0 0.0 0.0]';Me=[0 0 0]';
fe=[0 0 0]';le=[0 0 0]';

%% 计算
[Guess,t1,t2,y1,y2]=shootingMethod(qa,Fe,Me,fe,le,SP);

%% 函数
function [Guess,t1,t2,y1,y2]=shootingMethod(qa,Fe,Me,fe,le,SP)
    dGuess=eye(10)*1e-5; 
    lambda=5e-10; % Jacobian damping
    eps=1e-5; % residue tolerance

    nc=[0 0 0]';mc=[0 0 0]';
    Guess1=[nc;mc]; 
    rod_number = size(SP.ps,3);
    Guess1_length = size(Guess1, 1);
    Guess = zeros(rod_number * Guess1_length, 1);
    Guess_length = size(Guess,1);
    
    for i = 1 : rod_number
        Guess((1 + (i - 1) *Guess1_length):(i * Guess1_length)) = Guess1;
    end
    % guessed base force moment strain
    [Rsd,t1,y1]=forShooting(Guess,qa,Fe,Me,fe,le,SP);
    Rsd_length = max(size(Rsd));
    J=zeros(Rsd_length,Guess_length);
    tic
    while(norm(Rsd)>eps)
        disp(['residue0 = ' num2str(norm(Rsd))]);
    
        for i=1:Guess_length % finite differencing for Jacobian of initial guess
            [Rsd_]=forShooting(Guess+dGuess(:,i),qa,Fe,Me,fe,le,SP);
            J(:,i)=(Rsd_-Rsd)/norm(dGuess(:,i));
        end
        Guess=Guess-1/(J'*J+lambda*eye(10))*J'*(Rsd); % update guess
        [Rsd,t1,y1,t2,y2]=forShooting(Guess,qa,Fe,Me,fe,le,SP); % number IVP of equal to number of guessed values
            

    end
    toc
end