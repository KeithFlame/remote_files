function qa = setQA(dQA)
    qa1 = dQA(1:6);qa2 = dQA(7:12);
    QA_limit = [20 1 0.1 0.1 0.1 0.1]'/1000;
    
    t = abs(qa1);
    a=find((t-QA_limit)>0);
    qae1 = qa1;
    qae1(a)=sign(qa1(a)).*QA_limit;

    t = abs(qa2);
    a=find((t-QA_limit)>0);
    qae2 = qa2;
    qae2(a)=sign(qa2(a)).*QA_limit;
    
    qa=[qae1;qae2];
end