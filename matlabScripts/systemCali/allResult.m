%---------direct result and optimaled result--------------------%
% calculating precision result.
% ----Info
% By Keith W.
% Date 20201110
% Ver c1.0
% 
%-------------------------------------------------------------------------%

name='59_1';
[pos_err,dir_err]=testConfigErr(name,1);
figure;
subplot(1,2,1);
hold on;
stem(pos_err,'g');
stem(dir_err,'r');
title(['共',num2str(size(pos_err,1)),'构型,',name,'位置误差为',num2str(mean(pos_err)),'mm，姿态误差为',num2str(mean(dir_err)),'°。']);
legend('pos err','dir err');
hold off;
[pos_err0,dir_err0]=testConfigErr(name,0);
subplot(1,2,2);
hold on;
stem(pos_err0,'g');
stem(dir_err0,'r');
title(['优化后',name,'位置误差为',num2str(mean(pos_err0)),'mm，姿态误差为',num2str(mean(dir_err0)),'°。']);
legend('pos err','dir err');
hold off;


name='33_2';
[pos_err,dir_err]=testConfigErr(name,1);
figure;
subplot(1,2,1);
hold on;
stem(pos_err,'g');
stem(dir_err,'r');
title(['共',num2str(size(pos_err,1)),'构型,',name,'位置误差为',num2str(mean(pos_err)),'mm，姿态误差为',num2str(mean(dir_err)),'°。']);
legend('pos err','dir err');
hold off;
[pos_err0,dir_err0]=testConfigErr(name,0);
subplot(1,2,2);
hold on;
stem(pos_err0,'g');
stem(dir_err0,'r');
title(['优化后',name,'位置误差为',num2str(mean(pos_err0)),'mm，姿态误差为',num2str(mean(dir_err0)),'°。']);
legend('pos err','dir err');
hold off;


name='33_4';
[pos_err,dir_err]=testConfigErr(name,1);
figure;
subplot(1,2,1);
hold on;
stem(pos_err,'g');
stem(dir_err,'r');
title(['共',num2str(size(pos_err,1)),'构型,',name,'位置误差为',num2str(mean(pos_err)),'mm，姿态误差为',num2str(mean(dir_err)),'°。']);
legend('pos err','dir err');
hold off;
[pos_err0,dir_err0]=testConfigErr(name,0);
subplot(1,2,2);
hold on;
stem(pos_err0,'g');
stem(dir_err0,'r');
title(['优化后',name,'位置误差为',num2str(mean(pos_err0)),'mm，姿态误差为',num2str(mean(dir_err0)),'°。']);
legend('pos err','dir err');
hold off;


name='59_1';
[pos_err,dir_err]=testConfigErr(name,1);
figure;
subplot(1,2,1);
hold on;
stem(pos_err,'g');
stem(dir_err,'r');
title(['共',num2str(size(pos_err,1)),'构型,',name,'位置误差为',num2str(mean(pos_err)),'mm，姿态误差为',num2str(mean(dir_err)),'°。']);
legend('pos err','dir err');
hold off;
[pos_err0,dir_err0]=testConfigErr(name,0);
subplot(1,2,2);
hold on;
stem(pos_err0,'g');
stem(dir_err0,'r');
title(['优化后',name,'位置误差为',num2str(mean(pos_err0)),'mm，姿态误差为',num2str(mean(dir_err0)),'°。']);
legend('pos err','dir err');
hold off;


name='59_2';
[pos_err,dir_err]=testConfigErr(name,1);
figure;
subplot(1,2,1);
hold on;
stem(pos_err,'g');
stem(dir_err,'r');
title(['共',num2str(size(pos_err,1)),'构型,',name,'位置误差为',num2str(mean(pos_err)),'mm，姿态误差为',num2str(mean(dir_err)),'°。']);
legend('pos err','dir err');
hold off;
[pos_err0,dir_err0]=testConfigErr(name,0);
subplot(1,2,2);
hold on;
stem(pos_err0,'g');
stem(dir_err0,'r');
title(['优化后',name,'位置误差为',num2str(mean(pos_err0)),'mm，姿态误差为',num2str(mean(dir_err0)),'°。']);
legend('pos err','dir err');
hold off;


name='59_4';
[pos_err,dir_err]=testConfigErr(name,1);
figure;
subplot(1,2,1);
hold on;
stem(pos_err,'g');
stem(dir_err,'r');
title(['共',num2str(size(pos_err,1)),'构型,',name,'位置误差为',num2str(mean(pos_err)),'mm，姿态误差为',num2str(mean(dir_err)),'°。']);
legend('pos err','dir err');
hold off;
[pos_err0,dir_err0]=testConfigErr(name,0);
subplot(1,2,2);
hold on;
stem(pos_err0,'g');
stem(dir_err0,'r');
title(['优化后',name,'位置误差为',num2str(mean(pos_err0)),'mm，姿态误差为',num2str(mean(dir_err0)),'°。']);
legend('pos err','dir err');
hold off;


name='61_1';
[pos_err,dir_err]=testConfigErr(name,1);
figure;
subplot(1,2,1);
hold on;
stem(pos_err,'g');
stem(dir_err,'r');
title(['共',num2str(size(pos_err,1)),'构型,',name,'位置误差为',num2str(mean(pos_err)),'mm，姿态误差为',num2str(mean(dir_err)),'°。']);
legend('pos err','dir err');
hold off;
[pos_err0,dir_err0]=testConfigErr(name,0);
subplot(1,2,2);
hold on;
stem(pos_err0,'g');
stem(dir_err0,'r');
title(['优化后',name,'位置误差为',num2str(mean(pos_err0)),'mm，姿态误差为',num2str(mean(dir_err0)),'°。']);
legend('pos err','dir err');
hold off;


name='61_2';
[pos_err,dir_err]=testConfigErr(name,1);
figure;
subplot(1,2,1);
hold on;
stem(pos_err,'g');
stem(dir_err,'r');
title(['共',num2str(size(pos_err,1)),'构型,',name,'位置误差为',num2str(mean(pos_err)),'mm，姿态误差为',num2str(mean(dir_err)),'°。']);
legend('pos err','dir err');
hold off;
[pos_err0,dir_err0]=testConfigErr(name,0);
subplot(1,2,2);
hold on;
stem(pos_err0,'g');
stem(dir_err0,'r');
title(['优化后',name,'位置误差为',num2str(mean(pos_err0)),'mm，姿态误差为',num2str(mean(dir_err0)),'°。']);
legend('pos err','dir err');
hold off;


name='61_4';
[pos_err,dir_err]=testConfigErr(name,1);
figure;
subplot(1,2,1);
hold on;
stem(pos_err,'g');
stem(dir_err,'r');
title(['共',num2str(size(pos_err,1)),'构型,',name,'位置误差为',num2str(mean(pos_err)),'mm，姿态误差为',num2str(mean(dir_err)),'°。']);
legend('pos err','dir err');
hold off;
[pos_err0,dir_err0]=testConfigErr(name,0);
subplot(1,2,2);
hold on;
stem(pos_err0,'g');
stem(dir_err0,'r');
title(['优化后',name,'位置误差为',num2str(mean(pos_err0)),'mm，姿态误差为',num2str(mean(dir_err0)),'°。']);
legend('pos err','dir err');
hold off;


name='70_4';
[pos_err,dir_err]=testConfigErr(name,1);
figure;
subplot(1,2,1);
hold on;
stem(pos_err,'g');
stem(dir_err,'r');
title(['共',num2str(size(pos_err,1)),'构型,',name,'位置误差为',num2str(mean(pos_err)),'mm，姿态误差为',num2str(mean(dir_err)),'°。']);
legend('pos err','dir err');
hold off;
[pos_err0,dir_err0]=testConfigErr(name,0);
subplot(1,2,2);
hold on;
stem(pos_err0,'g');
stem(dir_err0,'r');
title(['优化后',name,'位置误差为',num2str(mean(pos_err0)),'mm，姿态误差为',num2str(mean(dir_err0)),'°。']);
legend('pos err','dir err');
hold off;



