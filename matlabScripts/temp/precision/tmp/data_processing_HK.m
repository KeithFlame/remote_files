function X = data_processing_HK(file_name)

init_data = load(file_name);
m = size(init_data,1);
block_size = m/5;
Pose = zeros(7,block_size);
for i = 1:block_size
%     if(i == init_data(5*i-4,1))
        if(0==init_data(5*i-4,4))
            Pose(7,i) = 100;
            continue;
        else
            Pose(7,i) = init_data(5*i-4,4);
        end
        T = init_data((5*i-3):(5*i),:);
        T = refineT(T);
        x = fromT2X(T);
        Pose(1:6,i)=x;

%     end
end
X=Pose;
end