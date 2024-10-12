data_path_1 = [pwd,'/pic_factory/test_0331_1'];
data_path_2 = [pwd,'/pic_factory/test_0331_2'];
data_path_3 = [pwd,'/pic_factory/test_0331_3'];

save_path_pln = [pwd,'/pic_factory/trial_1/plane_fig_pos'];

[T_tar_1,T_cur_1,pos_err_1,ang_err_1]=data_todo(data_path_1);
[T_tar_2,T_cur_2,pos_err_2,ang_err_2]=data_todo(data_path_2);
[T_tar_3,T_cur_3,pos_err_3,ang_err_3]=data_todo(data_path_3);


p1=T_cur_1(1:3,4,:);
p2 = reshape(p1,[3 size(T_cur_1,3)]);
figure;
hold on;axis equal;
plot3(p2(1,:),p2(2,:),p2(3,:),'c--');
plot3([p2(1,1) T_tar_1(1,4)],[p2(2,1) T_tar_1(2,4)],[p2(3,1) T_tar_1(3,4)],'r-')
