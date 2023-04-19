SL = [100 10 20 15 0.2 5 0.6 0 600]';
ksi=load('ksi.log');
Guess_fk = load('guess.log');
Guess_inv = load('guess_inv.log');
qa=load('qa.log');
pose=load('../pose.log');
target=load('target.log');
MBP1 = MultiBackboneParameter_keith;
MBP1 = MBP1.resetCalibrationPara(SL);
MBP1 = MBP1.refreshLso(qa(2)*1000);
MBP2 = MultiBackboneParameter_keith;
MBP2 = MBP2.resetCalibrationPara(SL);
MBP2 = MBP2.refreshLso(qa(8)*1000);
MBPF = [MBP1 MBP2];

ksi0 = [MBP1.Ke1(3,3) MBP1.Kb1(1,1) MBP1.Kb1(2,2) MBP1.Kb1(3,3)...
    MBP1.Ke1(3,3) MBP1.Kb1(1,1) MBP1.Kb1(2,2) MBP1.Kb1(3,3)...
    MBP1.K1 MBP1.K2 MBP2.K1 MBP2.K2]';
ksi_ratio = ksi ./ ksi0 - 1;
%% stiff 
Guess_sf = [Guess_fk;ksi_ratio];
T = fromQuat2T(pose);
[guess_sf,MBPF,~,~,~,~,y0,y1,y2,y3,ksi2]=shootingStiffOpt_keith(Guess_sf,qa,T,MBPF);

%% inverse kinematics
% Guess_inv = zeros(24,1);
% Guess_inv(1:6)=guess_sf(1:6);Guess_inv(9:18)=guess_sf(7:16);Guess_inv(21:24)=guess_sf(17:20);
[guess_inv,MBPF,~,~,~,~,y0,y1,y2,y3,QA]=shootingInvOpt_keith(Guess_inv,target,ksi2,MBPF);

%% forward kinematics
qa = qa+setQA(QA-qa);
[guess_fk,MBPF,~,~,~,~,y0,y1,y2,y3]=shootingFkOpt_keith(Guess_sf(1:20),qa,ksi2,MBPF);

%% write

psi1=Actuation2Psi_keith(qa(1:6),SL,MBP1);
psi2=Actuation2Psi_keith(qa(7:12),SL,MBP2);
psi_write = [psi1;psi2]';
Guess_write=guess_fk;
Guess_inv_write=guess_inv;
ksi_write = ksi2;
qa_write = qa;
fwrite1=fopen("../psi.log",'w');
fprintf(fwrite1,'%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f',psi_write);
close(fwrite1);
fwrite1=fopen("./guess.log",'w');
fprintf(fwrite1,['%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f' ...
    '\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f'],Guess_write);
close(fwrite1);
fwrite1=fopen("./ksi.log",'w');
fprintf(fwrite1,'%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f',ksi_write);
close(fwrite1);
fwrite1=fopen("./qa.log",'w');
fprintf(fwrite1,'%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f',qa_write);
close(fwrite1);
fwrite1=fopen("./guess_inv.log",'w');
fprintf(fwrite1,['%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f' ...
    '%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f\t%6f'],Guess_inv_write);
close(fwrite1);

