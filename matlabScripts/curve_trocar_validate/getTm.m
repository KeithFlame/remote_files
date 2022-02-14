% 获取2e坐标系下的marker坐标系。
x=[0 0 0];
options = optimoptions('fmincon','Algorithm','interior-point',"EnableFeasibilityMode",true); % 'Display','iter',,'PlotFcn','optimplotfirstorderopt','PlotFcn','optimplotfval'
options.StepTolerance=1e-25;
options.OptimalityTolerance=5e-6;
[p0,yp,exitflag_p]=fmincon('costFunc_Position',x,[],[],[],[],[],[],[],options);

% p0,
xhmax=[pi,pi,pi];
xhmin=[-pi,-pi,-pi];
[r0,yr,exitflag_r]=fmincon('costFunc_oritientation',x,[],[],[],[],xhmin,xhmax,[],options);

% r0,
R=eul2rotm(r0);
T=[R,p0';[0 0 0 1]]