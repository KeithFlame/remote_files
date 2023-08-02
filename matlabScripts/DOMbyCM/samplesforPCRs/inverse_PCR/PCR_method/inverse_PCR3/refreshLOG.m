function refreshLOG
Guess_write=load('guess_bk.log');
Guess_inv_write=load('guess_inv_bk.log');
ksi_write = load('ksi_bk.log');
qa_write = load('qa_bk.log');
fwrite1=fopen("./guess.log",'w');
fprintf(fwrite1,['%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f' ...
    '\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f'],Guess_write);
fclose(fwrite1);
fwrite1=fopen("./ksi.log",'w');
fprintf(fwrite1,'%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f',ksi_write);
fclose(fwrite1);
fwrite1=fopen("./qa.log",'w');
fprintf(fwrite1,'%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f',qa_write);
fclose(fwrite1);
fwrite1=fopen("./guess_inv.log",'w');
fprintf(fwrite1,['%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n' ...
    '%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f\n%6f'],Guess_inv_write);
fclose(fwrite1);
fwrite1=fopen("../isread_flag.log",'w');
fprintf(fwrite1,'%d',0);
fclose(fwrite1);
end