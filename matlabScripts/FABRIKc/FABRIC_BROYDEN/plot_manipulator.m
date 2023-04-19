function  [position,orientation,z_1b,z_1e,z_2b,z_2e,l1,l2,P_1j,P_2j]=plot_manipulator(psi)
%   根据构型和构型参数绘制机械臂
%   cofig：构型数（1~4）
%   psi：构型参数
    Lg=20;
    L10=40;
    L20=60;
    Lr0=20;
    %分配参数
    phi=psi(1);       
    Ls=psi(2);
    xita1=psi(3);
    L1=L10;
    Lr=Lr0;
    delta1=psi(4);
    L2=L20;
    xita2=psi(5);
    delta2=psi(6);
    %求解位姿
    [P_1b_1b1e,continuum1]=continuum_position(xita1,L1,delta1);     
    [P_2b_2b2e,continuum2]=continuum_position(xita2,L2,delta2);
    
    R_w_1b=[cos(phi) -sin(phi) 0;sin(phi) cos(phi) 0;0 0 1];
  
    R_1b_2b=continuum_oriention(xita1,delta1);
    R_2b_2e=continuum_oriention(xita2,delta2);
    
    R_w_2b=R_w_1b*R_1b_2b; 
    R_w_2e=R_w_2b*R_2b_2e;
   
    continuum1=R_w_1b*continuum1;
    continuum2=R_w_2b*continuum2;
    P_w_w1b=[0;0;Ls];
    P_1e_1e2b=[0;0;Lr];
    P_2e_2eg=[0;0;Lg];
    P_w_w1e= R_w_1b*P_1b_1b1e+P_w_w1b;
    P_w_w2b=P_w_w1e+R_w_2b*P_1e_1e2b;
    P_w_w2e=P_w_w2b+R_w_2b*P_2b_2b2e;
    P_w_wg=P_w_w2e+R_w_2e*P_2e_2eg;
    [~,n]=size(continuum1);
    [~,m]=size(continuum2);
    for i=1:n
        continuum1(:,i)=continuum1(:,i)+P_w_w1b;
    end
    for i=1:m
        continuum2(:,i)=continuum2(:,i)+P_w_w2b;
    end 
    
    %绘制三维图形
    plot3(continuum2(1,:),continuum2(2,:),continuum2(3,:),'r','linewidth',2);
    hold on;
    plot3([P_w_w2e(1),P_w_wg(1)],[P_w_w2e(2),P_w_wg(2)],[P_w_w2e(3),P_w_wg(3)],'k','linewidth',1);
    hold on;
    plot3([P_w_w1e(1),P_w_w2b(1)],[P_w_w1e(2),P_w_w2b(2)],[P_w_w1e(3),P_w_w2b(3)],'g','linewidth',2);
    hold on;
    plot3(continuum1(1,:),continuum1(2,:),continuum1(3,:),'b','linewidth',2);
    hold on;
    plot3([0,P_w_w1b(1)],[0,P_w_w1b(2)],[0,P_w_w1b(3)],'k','linewidth',2);
    
    plot_orientation(P_w_wg,R_w_2e,20);
%     hold off;
           
    axis equal;
    set(gca,'XLim',[-200 200]);
    set(gca,'YLim',[-200 200]);
    set(gca,'ZLim',[0 400]);
    grid;
    
    if xita1 == 0
        l1 = L1/2;
    else
        l1 = L1/xita1 * tan(xita1/2);                                       %连续体1段虚拟杆长
    end
    
    if xita2 == 0
        l2 = L2/2;
    else
        l2 = L2/xita2 * tan(xita2/2);                                       %连续体2段虚拟杆长
    end
    
    z_1b=R_w_1b(:,3);                                                       %连续体1段基Z轴    
    z_1e=R_1b_2b(:,3);                                                      %连续体1段末Z轴
    z_2b=z_1e;                                                              %连续体2段基Z轴
    z_2e=R_w_2e(:,3);                                                       %连续体2段末Z轴
    
    P_1j=P_w_w1b+l1*z_1b;                                                   %连续体1虚拟关节位置
    P_2j=P_w_w2b+l2*z_1b;                                                   %连续体2虚拟关节位置
    
    position=P_w_wg;
    orientation=R_w_2e;
end

