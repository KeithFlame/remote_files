SL = [100 10 20 15 0.1 5 0.6 0 600]';
ksi=[28353 0.0016 0.0016 0.0012 0 28353 0.0016 0.0016 0.0012 0 5 5];
Guess_fk = load('guess.log');
Guess_inv = load('guess_inv.log');
qa=load('qa.log');
pose=load('../pose.log')';
target=load('target.log')';
ksi=reshape(ksi,[12 1]);
Guess_fk=reshape(Guess_fk,[20 1]);
Guess_inv=reshape(Guess_inv,[24 1]);
qa=reshape(qa,[12 1]);
pose=reshape(pose,[7 1]);
target=reshape(target,[7 1]);
MBP1 = MultiBackboneParameter_keith;
MBP1 = MBP1.resetCalibrationPara(SL);
MBP1 = MBP1.resetLg(17.9495e-3);
MBP1 = MBP1.setLdo(18.8954e-3);
MBP1 = MBP1.refreshLso(qa(2)*1000);
MBP2 = MultiBackboneParameter_keith;
MBP2 = MBP2.resetCalibrationPara(SL);
MBP2 = MBP2.refreshLso(qa(8)*1000);
MBP2 = MBP2.resetLg(16.7079e-3);
MBP2 = MBP2.setLdo(19.4488e-3);
MBPF = [MBP1 MBP2];


T = fromQuat2T(pose);
target = fromQuat2T(target);
perr=norm(T(1:3,4)-target(1:3,4))*1000;
axang=rotm2axang(T(1:3,1:3)'*target(1:3,1:3));
aerr=axang(4)*180/pi;
err=[perr aerr]';
if(max(err)<1)
    disp("已经收敛");
    ee2= load("Ee_record.log");
    ee2 = ee2(end);
end
%% ======execute========
[guess_inv,MBPF,~,~,~,~,y0,y1,y2,y3,QA]=shootingInvOpt_keith(Guess_inv,target,ksi,MBPF);

qa = qa+setQA(QA-qa);
[guess_fk,MBPF,~,~,~,~,y0,y1,y2,y3,ee]=shootingFkOpt_keith(Guess_fk,qa,ksi,MBPF);
% [s1,s2] = setPlotData(y0,y1,y2,y3);
% figure;axis equal;grid on;xlabel('X');ylabel('Y');zlabel('Z');
% plotParallelContinuumRobot(s1,s2);
% view([0 -90])
T = eye(4);
T(1:3,1)=y3(end,4:6)';T(1:3,2)=y3(end,7:9)';
T(1:3,3)=y3(end,10:12)';T(1:3,4)=y3(end,1:3)';
%% write
% Qac=qa;
% Qac([2:6 8:12])=Qac([2:6 8:12])*1000;
% psi1=Actuation2Psi_keith(Qac(1:6),SL,MBP1);
% psi2=Actuation2Psi_keith(Qac(7:12),SL,MBP2);
% % psi1(2)=psi1(2)*1000;psi2(2)=psi2(2)*1000;
% psi_write = [psi1;psi2]';
% Guess_write=guess_fk;
% Guess_inv_write=guess_inv;
% ksi_write = ksi;
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
% fwrite1=fopen("../isread_flag.log",'w');
% fprintf(fwrite1,'%d',1);
% fclose(fwrite1);
% fwrite1=fopen("./qa_record.log",'a');
% fprintf(fwrite1,'%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\n',Qac);
% fclose(fwrite1);
% fwrite1=fopen("./Ee_record.log",'a');
% fprintf(fwrite1,'%d\n',ee);
% fclose(fwrite1);
% 
% % % quat=rotm2quat(T(1:3,1:3));
% % % vec= [T(1:3,4)'*1000 quat];
% % % fwrite1=fopen("../pose.log",'w');
% % % fprintf(fwrite1,'%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f',vec);
% % % fclose(fwrite1);