data = load('tar3.log');
len = size(data,1);
figure(1);hold on; axis equal;
grid on;
PlotAxis(50,eye(4));
axis([-100 100 -80 100 0 180])
xlabel("x");
ylabel("y");
zlabel("z");

r0= reshape(data(1,4:end),[3 3])';
axang=rotm2axang(r0);
ang_err = zeros(len,1);
for i = 1:len
    % hg = PlotAxis(30,data(i,:));
    % pause(0.02);
    % delete(hg);
    r0= reshape(data(i,4:end),[3 3])';
    axang=rotm2axang(r0);
    ang_err(i) = axang(4);
end
%%
close;
figure(2);hold on;
xlabel("time");
ylabel("values");
data2=load('aa33aa.log');
plot(ang_err,'c-');
plot(data2(:,2),'r-');
% plot(data2(:,3),'b--');
% plot(data2(:,4),'b-');
% plot(data2(:,5),'g--');
% plot(data2(:,6),'g-');
% legend("angular\_error","\phi","\theta_1","\delta_1","\theta_2","\delta_2");