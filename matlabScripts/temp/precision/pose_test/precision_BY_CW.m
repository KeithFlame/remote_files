% file_name='data_HK2.log';
% X_HK = data_processing_HK(file_name);
% X_CW = getData_CW;
% 
% figure(1);
% hold on;grid on;axis equal;
% xlabel("\itX (mm)");
% ylabel("\itY (mm)");
% zlabel("\itZ (mm)");
% axis([-150 150 -60 100 240 440])
% for i = 1:size(X_HK,2)
%     if(X_HK(7,i)<50)
%         T=fromX2T(X_HK(1:6,i));
%         PlotAxis(10,T);
%     end
% end
% set(gca,'FontName','Times New Roman','FontSize',14);

%% 
[X_cw,X_hk]=getAllData;
xcw20 = fromX2T(X_cw(:,20));
xhk20 = fromX2T(X_hk(:,20));
T0 = eye(4);T0(3,4) = 0;
figure(1);cla;
hold on;grid on;axis equal;
xlabel("\itX (mm)");
ylabel("\itY (mm)");
zlabel("\itZ (mm)");
% axis([-150 150 -60 100 240 440])
for i = 1:2
        Thk = xhk20\fromX2T(X_hk(:,i))*T0;
        PlotAxis(10,Thk);
        Tcw = xcw20\fromX2T(X_cw(:,i));
        PlotAxis(20,Tcw);
end
%%
options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
% x0=zeros(6,1)+1;x0(3)=-150;x0(6)=1;
% x0=[1.5454    0.5120 -138.9055 -517.9033 -735.3975   -5.3223]';
x0 =rand(6,1);x0(3)=x0(3)-150;x0(6)=1;
x0=[2.1013   -0.1627 -139.3472  104.4344  147.3742    1.2405]';
[x,fval,exitflag,output] = fmincon('costFunc_CWHK',x0,[],[],[],[],[],[],[],options);