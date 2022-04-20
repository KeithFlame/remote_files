function Q=getQ_keith
    q1=190:10:490;
    q2=190:10:490;
    q3=190:10:490;
    block_size=max(size(q1));
    Q=zeros(3,block_size^3);
    i=1;
    for i1=1:max(size(q1))
        for i2=1:max(size(q1))
            for i3=1:max(size(q1))
                Q(:,i)=[q1(i1);q2(i2);q3(i3)];
                i=i+1;
            end
        end
    end
end