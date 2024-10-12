SL = [99.2 10 19.4 12 0.1 5 0.6 0 500 0]';
% qa = [0 78 1.4501720609068767 0 0.83304055090469742 0.83304055090469742 0 72 0.74005858372053546 -1.2818190675814414 1.1379545549183929 -0.30491400401370428]'/1000;
qa=[ 100/180*pi 30 20/180*pi -35/180*pi 20/180*pi 145/180*pi ...
     100/180*pi 30 20/180*pi 35/180*pi 20/180*pi -145/180*pi]'/1000;
Guess_fk = zeros(20,1);
MBP1 = MultiBackboneParameter_keith;
MBP1 = MBP1.resetCalibrationPara(SL);
MBP1 = MBP1.resetLg(15e-3);
MBP1 = MBP1.setLdo(6.7608e-3);
MBP1 = MBP1.refreshLso(qa(2)*1000);
MBP2 = MultiBackboneParameter_keith;
MBP2 = MBP2.resetCalibrationPara(SL);
MBP2 = MBP2.refreshLso(qa(8)*1000);
MBP2 = MBP2.resetLg(15e-3);
MBP2 = MBP2.setLdo(18.5334e-3);
MBPF = [MBP1 MBP2];

ksi = [MBP1.Ke1(3,3) MBP1.Kb1(1,1) MBP1.Kb1(2,2) MBP1.Kb1(3,3) 2 ...
    MBP1.Ke1(3,3) MBP1.Kb1(1,1) MBP1.Kb1(2,2) MBP1.Kb1(3,3) 2 ...
    MBP1.K1 MBP2.K1]';

qa1=Psi2Actuation_keith(qa(1:6)*1e3,SL,MBP1);qa1(1)=qa1(1)*1e3;
qa2=Psi2Actuation_keith(qa(7:12)*1e3,SL,MBP1);qa2(1)=qa2(1)*1e3;

qa=[qa1;qa2]/1000;

Guess_fk=[			1.7968437804134541	
	0.041894186235452519	
			0.35784084851871417	
			0.089112590806342304	
			-0.0060974077743350683	
			-0.019437454881808099	
			0.00086431770010189971	
		-0.00072275200309568615	
			-0.00033333634334053596	
			-8.4831939294425376e-05	
			-1.7968437804134541	
			-0.041894186235510542	
			-0.35784084851868497	
			0.076057083324827940	
			-0.0020715724551797474	
		0.050172557570044143	
			0.00086521223644585093	
			0.00072307560092951979	
			-7.8942208139833755e-05	
			-0.00033714446870134292];
Guess_fk(2:end)=0;
[guess_fk,MBPF,~,~,~,~,y0,y1,y2,y3]=shootingFkOpt_keith(Guess_fk,qa,ksi,MBPF);
[s1,s2] = setPlotData(y0,y1,y2,y3);
figure;axis equal;grid on;xlabel('X');ylabel('Y');zlabel('Z');
plotParallelContinuumRobot(s1,s2);
T = eye(4);
T(1:3,1)=y3(end,4:6)';T(1:3,2)=y3(end,7:9)';
T(1:3,3)=y3(end,10:12)';T(1:3,4)=y3(end,1:3)';

dr12=[-36.468 -137.416 -3.163];
dr12 = fromaxang2rotm(dr12);
T(1:3,1:3)=T(1:3,1:3)*dr12