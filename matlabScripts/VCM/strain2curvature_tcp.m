function strain2curvature_tcp()
%---for tcp reading
tcpdata=importdata('./0924/client.txt');
[delta_comp]=fbgCalibrate();
straindata=tcpdata.data(:,130:183);
num=length(straindata);
U=zeros(num,21);
for i=1:num
    Ch1=straindata(i,1:18);
    Ch2=straindata(i,19:36);
    Ch3=straindata(i,37:54);
    [u]=strain2curvature(delta_comp,Ch1,Ch2,Ch3);
    U(i,:)=[u(1,12:18) u(2,12:18) u(3,12:18)];
end


% fop = fopen( './0924/curvatures.txt', 'wt' );
% [M,N] = size(U);
% for m = 1:M
%     for n = 1:N
%         fprintf( fop, ' %s', mat2str( U(m,n) ) );
%     end
%     fprintf(fop, '\n' );
% end
% back = fclose( fop );


figure(1)
hold on;
plot(U(:,1));
plot(U(:,2));
plot(U(:,3));
plot(U(:,4));
plot(U(:,5));
plot(U(:,6));


%read force
figure(2)
%forcedata=importdata('./0924/Force2_2.txt');
F2=load('./0924/Force2_2.mat');
forcedata=F2.Force_;
num_force = length(forcedata);
t=0:1:num_force-1;
t=t*10e-3;
hold on;
plot(t,forcedata(:,1),'-b');
plot(t,forcedata(:,2),'-r');




end