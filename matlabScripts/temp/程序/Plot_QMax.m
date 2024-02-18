function Plot_QMax(RCM_Y_list,Q_max_Jj_tab,Q_max_Vx_tab,Q_max_Vy_tab)
Q1_max_Jj=Q_max_Jj_tab(:,1);Q2_max_Jj=Q_max_Jj_tab(:,2);Q3_max_Jj=Q_max_Jj_tab(:,3);Q4_max_Jj=Q_max_Jj_tab(:,4);
Q5_max_Jj=Q_max_Jj_tab(:,5);Q6_max_Jj=Q_max_Jj_tab(:,6);Q7_max_Jj=Q_max_Jj_tab(:,7);

Q1_max_Vx=Q_max_Vx_tab(:,1);Q2_max_Vx=Q_max_Vx_tab(:,2);Q3_max_Vx=Q_max_Vx_tab(:,3);Q4_max_Vx=Q_max_Vx_tab(:,4);
Q5_max_Vx=Q_max_Vx_tab(:,5);Q6_max_Vx=Q_max_Vx_tab(:,6);Q7_max_Vx=Q_max_Vx_tab(:,7);

Q1_max_Vy=Q_max_Vy_tab(:,1);Q2_max_Vy=Q_max_Vy_tab(:,2);Q3_max_Vy=Q_max_Vy_tab(:,3);Q4_max_Vy=Q_max_Vy_tab(:,4);
Q5_max_Vy=Q_max_Vy_tab(:,5);Q6_max_Vy=Q_max_Vy_tab(:,6);Q7_max_Vy=Q_max_Vy_tab(:,7);
%%
figure(1)
set(gcf,'Position',[100 100 270*3 100*3]);%左下角位置，宽高，这里的260正好是7cm，适合半个word页
subplot(1,2,1)
grid on;hold on;
plot(RCM_Y_list,Q1_max_Jj,'-squareb','LineWidth',1);
plot(RCM_Y_list,Q3_max_Jj,'-squarec','LineWidth',1);
plot(RCM_Y_list,Q5_max_Jj,'-squarek','LineWidth',1);
plot(RCM_Y_list,Q7_max_Jj,'-squarem','LineWidth',1);
xlabel('RCM Y坐标')  %x轴坐标描述
ylabel('Qmax进给(°/s)') %y轴坐标描述
legend('Q1','Q3','Q5','Q7','location','Best');   %右上角标注

subplot(1,2,2)
grid on;hold on;
plot(RCM_Y_list,Q2_max_Jj,'-or','LineWidth',1);
plot(RCM_Y_list,Q4_max_Jj,'-og','LineWidth',1);
plot(RCM_Y_list,Q6_max_Jj,'-oc','LineWidth',1);
xlabel('RCM Y坐标')  %x轴坐标描述
ylabel('Qmax 进给(°/s)') %y轴坐标描述
legend('Q2','Q4','Q6','location','Best');   %右上角标注

%%
figure(2)
set(gcf,'Position',[100 100 270*3 100*3]);%左下角位置，宽高，这里的260正好是7cm，适合半个word页
subplot(1,2,1)
grid on;hold on;
plot(RCM_Y_list,Q1_max_Vx,'-squareb','LineWidth',1);
plot(RCM_Y_list,Q3_max_Vx,'-squarec','LineWidth',1);
plot(RCM_Y_list,Q5_max_Vx,'-squarek','LineWidth',1);
plot(RCM_Y_list,Q7_max_Vx,'-squarem','LineWidth',1);
xlabel('RCM Y坐标')  %x轴坐标描述
ylabel('Qmax X方向(°/s)') %y轴坐标描述
legend('Q1','Q3','Q5','Q7','location','Best');   %右上角标注

subplot(1,2,2)
grid on;hold on;
plot(RCM_Y_list,Q2_max_Vx,'-or','LineWidth',1);
plot(RCM_Y_list,Q4_max_Vx,'-og','LineWidth',1);
plot(RCM_Y_list,Q6_max_Vx,'-oc','LineWidth',1);
xlabel('RCM Y坐标')  %x轴坐标描述
ylabel('Qmax X方向(°/s)') %y轴坐标描述
legend('Q2','Q4','Q6','location','Best');   %右上角标注

%%
figure(3)
set(gcf,'Position',[100 100 270*3 100*3]);%左下角位置，宽高，这里的260正好是7cm，适合半个word页
subplot(1,2,1)
grid on;hold on;
plot(RCM_Y_list,Q1_max_Vy,'-squareb','LineWidth',1);
plot(RCM_Y_list,Q3_max_Vy,'-squarec','LineWidth',1);
plot(RCM_Y_list,Q5_max_Vy,'-squarek','LineWidth',1);
plot(RCM_Y_list,Q7_max_Vy,'-squarem','LineWidth',1);
xlabel('RCM Y坐标')  %x轴坐标描述
ylabel('Qmax Y方向(°/s)') %y轴坐标描述
legend('Q1','Q3','Q5','Q7','location','Best');   %右上角标注

subplot(1,2,2)
grid on;hold on;
plot(RCM_Y_list,Q2_max_Vy,'-or','LineWidth',1);
plot(RCM_Y_list,Q4_max_Vy,'-og','LineWidth',1);
plot(RCM_Y_list,Q6_max_Vy,'-oc','LineWidth',1);
xlabel('RCM Y坐标')  %x轴坐标描述
ylabel('Qmax Y方向(°/s)') %y轴坐标描述
legend('Q2','Q4','Q6','location','Best');   %右上角标注
end