file_name='data_HK4.log';
X_HK4 = data_processing_HK(file_name);
file_name='data_HK5.log';
X_HK5 = data_processing_HK(file_name);

X_HK = [X_HK4 X_HK5];
figure(8);hold on; grid on;axis equal;
xlabel("\itx (mm)");
ylabel("\ity (mm)");
zlabel("\itz (mm)");
T0=eye(4);T0(1,4)=-50;
for i = 1:size(X_HK,2)
    T = fromX2T(X_HK(1:6,i));
    if(i>27&&i<55)
        T = T0*T;
    end
    PlotAxis(10,T);
end
set(gca,"FontSize",16);
set(gca,"FontName","times new roman")