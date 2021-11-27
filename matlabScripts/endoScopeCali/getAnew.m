function getAnew
    procId=load('./arm_id.log');
    path=["./calibration/"+procId+"/cam_params/cam_params.sr"];
    fidin=fopen(path,'r');
    fout1=fopen('./autoCali_20210221/newCost/logRead/logRead/A_new.log','w');
    fout2=fopen('./autoCali_20210221/newCost/logRead/logRead/t.log','w');
    nline=0;
    while ~feof(fidin) % 判断是否为文件末尾
        tline=fgetl(fidin); % 从文件读行
        
        if nline==3
            tem=strsplit(tline,',');
            
            B=str2num(tem{2});
            
            fprintf(fout2,'%6.6f',B);
        end
        
        if nline>28 && nline<32
            tem=strsplit(tline,',');
            A=str2num(tem{1});
            B=str2num(tem{2});C=str2num(tem{3});
            tt=[A B C];
            fprintf(fout1,'%6.6f	%6.6f	%6.6f\n',tt);
        end
        nline=nline+1;
    end
    fclose(fidin);
    fclose(fout1);
    fclose(fout2);
end

    