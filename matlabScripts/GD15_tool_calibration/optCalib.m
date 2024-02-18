%% === optimization-based calibration === %%
clc;clear;
load('data\T1m.mat');
load('data\T2m.mat');
load('data\T3m.mat');
load('data\T4m.mat');
load('data\T5m.mat');
Poses1(:,:,1)=T1m;Poses1(:,:,2)=T2m;Poses1(:,:,3)=T3m;Poses1(:,:,4)=T4m;Poses1(:,:,5)=T5m;
targetPoses;
Poses0(:,:,1)=T1;Poses0(:,:,2)=T2;Poses0(:,:,3)=T3;Poses0(:,:,4)=T4;Poses0(:,:,5)=T5;
ga=0;La=200;L1=100;Lr=10;L2=20.4;k11=5;k12=5;k21=1.4;k22=1.4;rp=11;
x=[ga La L1 Lr L2 k11 k12 k21 k22 rp]';
% x=[3.1144 221.3976 98.6107 8.3512 20.3015 6.0250 5.0001 1.7467 1.3993 10.6620]';
W=diag([1 300 1 1 1 1 1 1 1 1]);
[Q,Psi_ref,flag]=inverseCalib(x,Poses0);
rsd=calcError(Poses1,Poses0);
disp(['error:' num2str(norm(rsd(1:3))) ' ' num2str(norm(rsd(4:6))) ' ' num2str(norm(rsd(7:9))) ' ' ...
    num2str(norm(rsd(10:12))) ' ' num2str(norm(rsd(13:15))) ' ' num2str(norm(rsd(16:18))) ' ' ...
    num2str(norm(rsd(19:21))) ' ' num2str(norm(rsd(22:24))) ' ' num2str(norm(rsd(25:27))) ' ' ...
    num2str(norm(rsd(28:30)))]);
N=length(x);
j=1;
iter_max=200;
ep=zeros(iter_max,1);
ew=zeros(iter_max,1);
ep(j)=mean([norm(rsd(1:3)) norm(rsd(4:6)) norm(rsd(7:9)) norm(rsd(10:12)) norm(rsd(13:15))]);
ew(j)=mean([norm(rsd(16:18)) norm(rsd(19:21)) norm(rsd(22:24)) norm(rsd(25:27)) norm(rsd(28:30))]);
if(flag==1)
    dx0=1e-3;
    dx=diag(ones(N,1))*dx0;
    J=zeros(30,N);
    lambda = 1e2;
    status=limitX(x);
    while(j<iter_max)
        for i=1:N
            [Poses_a]=forwardCalib(Q,x,Psi_ref);
            [Poses_b]=forwardCalib(Q,x+dx(:,i),Psi_ref);
            e_a=calcError(Poses1,Poses_a);
            e_b=calcError(Poses1,Poses_b);
            J(:,i)=(e_b-e_a)/dx0;
        end
        
        J_=contractMat(J*W,status);
        dim=sum(status);
        x_incr=-inv(J_'*J_+lambda*eye(dim))*J_'*(rsd);
        x_incr_=invContractVec(x_incr,status);
        x_=x+x_incr_;
        status_ = limitX(x_);
        status = double(status&status_);
        x=x+diag(status_)*x_incr_;
        Poses = forwardCalib(Q,x,Psi_ref);
        rsd=calcError(Poses1,Poses);
        disp(['error:' num2str(norm(rsd(1:3))) ' ' num2str(norm(rsd(4:6))) ' ' num2str(norm(rsd(7:9))) ' ' ...
            num2str(norm(rsd(10:12))) ' ' num2str(norm(rsd(13:15))) ' ' num2str(norm(rsd(16:18))) ' ' ...
            num2str(norm(rsd(19:21))) ' ' num2str(norm(rsd(22:24))) ' ' num2str(norm(rsd(25:27))) ' ' ...
            num2str(norm(rsd(28:30)))]);
        j=j+1;
        ep(j)=mean([norm(rsd(1:3)) norm(rsd(4:6)) norm(rsd(7:9)) norm(rsd(10:12)) norm(rsd(13:15))]);
        ew(j)=mean([norm(rsd(16:18)) norm(rsd(19:21)) norm(rsd(22:24)) norm(rsd(25:27)) norm(rsd(28:30))]);
    end
    
else
end
figure();hold on
plot(ep);
plot(ew);
disp(['mean ep=' num2str(ep(200)) ', mean ew=' num2str(ew(200))]);
%% ==
function [rsd] =calcError(Poses1,Poses)
rsd=zeros(30,1);
rsd(1:3)=(Poses(1:3,4,1)-Poses1(1:3,4,1));
rsd(4:6)=(Poses(1:3,4,2)-Poses1(1:3,4,2));
rsd(7:9)=(Poses(1:3,4,3)-Poses1(1:3,4,3));
rsd(10:12)=(Poses(1:3,4,4)-Poses1(1:3,4,4));
rsd(13:15)=(Poses(1:3,4,5)-Poses1(1:3,4,5));
% rsd(16:18)=(RotMatToEuler(Poses(1:3,1:3,1))-RotMatToEuler(Poses1(1:3,1:3,1)));
% rsd(19:21)=(RotMatToEuler(Poses(1:3,1:3,2))-RotMatToEuler(Poses1(1:3,1:3,2)));
% rsd(22:24)=(RotMatToEuler(Poses(1:3,1:3,3))-RotMatToEuler(Poses1(1:3,1:3,3)));
% rsd(25:27)=(RotMatToEuler(Poses(1:3,1:3,4))-RotMatToEuler(Poses1(1:3,1:3,4)));
% rsd(28:30)=(RotMatToEuler(Poses(1:3,1:3,5))-RotMatToEuler(Poses1(1:3,1:3,5)));
rsd(16:18)=(Logm(Poses(1:3,1:3,1))-Logm(Poses1(1:3,1:3,1)))/pi*180;
rsd(19:21)=(Logm(Poses(1:3,1:3,2))-Logm(Poses1(1:3,1:3,2)))/pi*180;
rsd(22:24)=(Logm(Poses(1:3,1:3,3))-Logm(Poses1(1:3,1:3,3)))/pi*180;
rsd(25:27)=(Logm(Poses(1:3,1:3,4))-Logm(Poses1(1:3,1:3,4)))/pi*180;
rsd(28:30)=(Logm(Poses(1:3,1:3,5))-Logm(Poses1(1:3,1:3,5)))/pi*180;
end

function [c]=limitX(x)
x_upper=[10 600 108 12 22 8 8 4 4 14]';
x_lower=[-10 150 92 8 18 1 1 0.01 0.01 10]';
c1=x>x_upper;
c2=x<x_lower;
c=~(c1|c2);
c=double(c);
end

function [J_]=contractMat(J,status)
cnt=1;
N=length(status);
J_=[];
for i=1:N
    if(status(i)==1)
        J_(:,cnt)=J(:,i);
        cnt=cnt+1;
    else
    end
end
end
function [v_]=contractVec(v,status)
cnt=1;
N=length(status);
for i=1:N
    if(status(i)==1)
        v_(cnt)=v(i);
        cnt=cnt+1;
    else
    end
end
v_=v_';
end
function [v_]=invContractVec(v,status)
N=length(status);
cnt=0;
for i=1:N
    if(status(i)==1)
        v_(i)=v(i-cnt);
    else
        v_(i)=0;
        cnt=cnt+1;
    end
end
v_=v_';
end