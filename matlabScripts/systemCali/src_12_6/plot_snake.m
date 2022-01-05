function [Tall, dis_err, ang_err]=plot_snake(Tm,psi,SP)


structure_para=SP;
seg_len = structure_para(1:4)'*1e-3;
bend_in_trocar = structure_para(11)*1e-3;
offset_zero = -3.7*1e-3+structure_para(10)*1e-3;
zeta = structure_para(5);

 %% trocar outport w.r.t trocar world
T_ch_tr =getPortChannel;

%%
Tm(1:3,4)=Tm(1:3,4)/1000;
psi_ = [psi(2) psi(1)/1000 psi(3) -psi(4) psi(5) -psi(6)]';
Tg(1:4,1:4) = T_ch_tr\Tm(1:4,1:4);
hold on;
PlotAxis(0.01,eye(4));%trocar base considering tau
PlotAxis(0.01,inv(T_ch_tr));%trocar world frame    
PlotCircle(5.0e-3,[0 0 0],[0 0 1]);
PlotAxis(0.005,Tg);%actual target 
Tall=PlotSnake_trocar(psi_, seg_len, zeta, 0, 0, bend_in_trocar,offset_zero);
PlotAxis(0.005,Tall.T_tipg);%theoretical target
if(Tg(3,4)==0)
    fprintf('NULL matrix!\n');
    title('NULL matrix!');
    dis_err=0;
    ang_err=0;
else
    dis_err=norm(Tall.T_tipg(1:3,4)-Tg(1:3,4))*1000;
%     dis_err=(Tall.T_tipg(3,4)-Tg(3,4))*1000;
    ang_err=acosd(dot(Tall.T_tipg(1:3,3),Tg(1:3,3)));
    
    
    err=[dis_err,ang_err];
    title(['pos_err: ',num2str(err(1)),'mm, ang_err: ',num2str(err(2)),'°']);
    fprintf("pos_err: %fmm, ang_err: %f°\n",err);
end

end