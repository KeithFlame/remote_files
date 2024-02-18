fwrite1=fopen("../isread_flag.log",'w');
fprintf(fwrite1,'%d',0);
fclose(fwrite1);
t =load("../isread_flag.log");
if(t==0)
    
else
    disp("还没动！！！")
    return;
end
SL = [100 10 20 15 0.1 5 0.6 0 600 0]';
LgLdo = [12.5055   25.3269   17.9274    8.4355]';
ksi=load('ksi.log');
Guess_fk = load('guess.log');
Guess_inv = load('guess_inv.log');
Guess_sf = load('guess_sf.log')*1;
qa=load('qa.log');
pose=load('../pose.log')';
target=load('target.log')';
ksi=reshape(ksi,[12 1]);
Guess_fk=reshape(Guess_fk,[20 1]);
Guess_inv=reshape(Guess_inv,[24 1]);
Guess_sf=reshape(Guess_sf,[32 1]);
qa=reshape(qa,[12 1]);
pose=reshape(pose,[7 1]);
target=reshape(target,[7 1]);
MBP1 = MultiBackboneParameter_keith;
MBP1 = MBP1.resetCalibrationPara(SL);
MBP1 = MBP1.resetLg(LgLdo(1)*1e-3);
MBP1 = MBP1.setLdo(LgLdo(2)*1e-3);
MBP1 = MBP1.refreshLso(qa(2)*1000);
MBP2 = MultiBackboneParameter_keith;
MBP2 = MBP2.resetCalibrationPara(SL);
MBP2 = MBP2.refreshLso(qa(8)*1000);
MBP2 = MBP2.resetLg(LgLdo(3)*1e-3);
MBP2 = MBP2.setLdo(LgLdo(4)*1e-3);
MBPF = [MBP1 MBP2];

ksi0 = [MBP1.Ke1(3,3) MBP1.Kb1(1,1) MBP1.Kb1(2,2) MBP1.Kb1(3,3) 1 ...
    MBP1.Ke1(3,3) MBP1.Kb1(1,1) MBP1.Kb1(2,2) MBP1.Kb1(3,3) 1 ...
    MBP1.K1 MBP2.K1]';
ksi_ratio = (ksi-ksi0) ./ ksi0;
ksi_ratio([5 10])=ksi([5 10]);
T = fromQuat2T(pose);
target = fromQuat2T(target);
perr=norm(T(1:3,4)-target(1:3,4))*1000;
axang=rotm2axang(T(1:3,1:3)'*target(1:3,1:3));
aerr=axang(4)*180/pi;
err=[perr aerr]';
if(max(err)<2)
    disp("已经收敛");
    return;
end
%% stiff 
% Guess_sf = [Guess_fk;ksi_ratio];
tic;
[guess_sf,MBPF,~,~,~,~,y0,y1,y2,y3,ksi2]=shootingStiffOpt_keith(Guess_sf,qa,T,MBPF);

%% inverse kinematics
% Guess_inv = zeros(24,1);
% Guess_inv(1:6)=guess_sf(1:6);Guess_inv(9:18)=guess_sf(7:16);Guess_inv(21:24)=guess_sf(17:20);

[guess_inv,MBPF,~,~,~,~,y0,y1,y2,y3,QA,ee]=shootingInvOpt_keith(Guess_inv,target,ksi2,MBPF);

%% forward kinematics
qa = qa+setQA(QA-qa);
toc;
% [Guess_fk,MBPF,~,~,~,~,y0,y1,y2,y3,ee]=shootingFkOpt_keith(Guess_fk,qa,ksi2,MBPF);

% % % % [Guess_fk,MBPF,~,~,~,~,y0,y1,y2,y3]=shootingFkOpt_keith(Guess_sf(1:20),qa,ksi2,MBPF);
% % [s1,s2] = setPlotData(y0,y1,y2,y3);
% % figure;axis equal;grid on;xlabel('X');ylabel('Y');zlabel('Z');
% % plotParallelContinuumRobot(s1,s2);
% % view([0 -90])
% % T = eye(4);
% % T(1:3,1)=y3(end,4:6)';T(1:3,2)=y3(end,7:9)';
% % T(1:3,3)=y3(end,10:12)';T(1:3,4)=y3(end,1:3)';

%% write
% Qac=qa;
% Qac([2:6 8:12])=Qac([2:6 8:12])*1000;
% psi1=Actuation2Psi_keith(Qac(1:6),SL,MBP1);
% psi2=Actuation2Psi_keith(Qac(7:12),SL,MBP2);
% % psi1(2)=psi1(2)*1000;psi2(2)=psi2(2)*1000;
% psi_write = [psi1;psi2]';
% Guess_write=Guess_fk;
% Guess_inv_write=guess_inv;
% Guess_sf_write=guess_sf;
% ksi_write = ksi2;
% qa_write = qa;
% fwrite1=fopen("../psi.log",'w');
% fprintf(fwrite1,'%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f',psi_write);
% fclose(fwrite1);
% fwrite1=fopen("./guess.log",'w');
% fprintf(fwrite1,['%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f' ...
%     '\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f'],Guess_write);
% fclose(fwrite1);
% fwrite1=fopen("./ksi.log",'w');
% fprintf(fwrite1,'%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f',ksi_write);
% fclose(fwrite1);
% fwrite1=fopen("./qa.log",'w');
% fprintf(fwrite1,'%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f',qa_write);
% fclose(fwrite1);
% fwrite1=fopen("./guess_inv.log",'w');
% fprintf(fwrite1,['%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n' ...
%     '%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f'],Guess_inv_write);
% fclose(fwrite1);
% fwrite1=fopen("./guess_sf.log",'w');
% fprintf(fwrite1,['%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n' ...
%     '%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n' ...
%     '%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f'],Guess_sf_write);
% fclose(fwrite1);
% fwrite1=fopen("../isread_flag.log",'w');
% fprintf(fwrite1,'%d',1);
% fclose(fwrite1);
% fwrite1=fopen("./qa_record.log",'a');
% fprintf(fwrite1,'%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\n',Qac);
% fclose(fwrite1);
% % % fwrite1=fopen("./Ee_record.log",'a');
% % % fprintf(fwrite1,'%6f\n',ee);
% % % fclose(fwrite1);