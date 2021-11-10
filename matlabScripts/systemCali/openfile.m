clc;clear;

file_path = '..\tests\1019\pose_data_set1.raw';
fid=fopen(file_path);
tline = fgetl(fid);
count=1;
odd=1;
while ischar(tline)
    mline=str2num(tline);
    if(~isempty(mline))
        if(odd==1)
            p1=mline(1:3)';
            odd=0;
        else
            p2=mline(1:3)';
            if(length(mline)<7)
                R_raw = eye(3);
            else
                R_raw=quat2rotm(mline(4:7));
            end
            odd=1;
            z=p2-p1;
            z=z/norm(z);
            xd=R_raw(1:3,1);
            a=z'*xd;
            x=xd-a*z;
            x=x/norm(x);
            y=cross(z,x);
            T_m_c(1:4,1:4,count) = [x y z p1;0 0 0 1];
            count=count+1;
        end
    end

    disp(tline)
    tline = fgetl(fid);
    
end
fclose(fid);