function [T1,T2,T3]=getData

persistent T1_;
persistent T2_;
persistent T3_;
if(isempty(T1_)||isempty(T2_)||isempty(T3_))
    %"#"	"t"	"keith_0104_2 X"	"keith_0104_2 Y"	"keith_0104_2 Z"	
    % "keith_0104_2 R11"	"keith_0104_2 R12"	"keith_0104_2 R13"	
    % "keith_0104_2 R21"	"keith_0104_2 R22"	"keith_0104_2 R23"	
    % "keith_0104_2 R31"	"keith_0104_2 R32"	"keith_0104_2 R33"	
    % 
    % "keith_0104_3 X"	"keith_0104_3 Y"	"keith_0104_3 Z"	
    % "keith_0104_3 R11"	"keith_0104_3 R12"	"keith_0104_3 R13"	
    % "keith_0104_3 R21"	"keith_0104_3 R22"	"keith_0104_3 R23"	
    % "keith_0104_3 R31"	"keith_0104_3 R32"	"keith_0104_3 R33"	
    
    tem=load('data\24_D2.raw');
    block_size=size(tem,1);
    temT1=zeros(4,4,block_size);
    temT2=zeros(4,4,block_size);
    T1_=zeros(8,4,block_size);
    for i = 1 :block_size
        temT1(:,:,i)=[
            tem(i,6) tem(i,7) tem(i,8) tem(i,3)
            tem(i,9) tem(i,10) tem(i,11) tem(i,4)
            tem(i,12) tem(i,13) tem(i,14) tem(i,5)
            0 0 0 1];
        temT2(:,:,i)=[
            tem(i,18) tem(i,19) tem(i,20) tem(i,15)
            tem(i,21) tem(i,22) tem(i,23) tem(i,16)
            tem(i,24) tem(i,25) tem(i,26) tem(i,17)
            0 0 0 1];
    end
    T1_(1:4,:,:)=temT1;
    T1_(5:8,:,:)=temT2;
    
    % "#"	"t"	"keith_0104_3 X"	"keith_0104_3 Y"	"keith_0104_3 Z"	
    % "keith_0104_3 R11"	"keith_0104_3 R12"	"keith_0104_3 R13"	
    % "keith_0104_3 R21"	"keith_0104_3 R22"	"keith_0104_3 R23"	
    % "keith_0104_3 R31"	"keith_0104_3 R32"	"keith_0104_3 R33"	
    % 
    % "keith_0104_4 X"	"keith_0104_4 Y"	"keith_0104_4 Z"	
    % "keith_0104_4 R11"	"keith_0104_4 R12"	"keith_0104_4 R13"	
    % "keith_0104_4 R21"	"keith_0104_4 R22"	"keith_0104_4 R23"	
    % "keith_0104_4 R31"	"keith_0104_4 R32"	"keith_0104_4 R33"	

    tem=load('data\l=0,step=20_.raw');
    block_size=size(tem,1);
    temT1=zeros(4,4,block_size);
    temT2=zeros(4,4,block_size);
    T2_=zeros(8,4,block_size);
    for i = 1 :block_size
        temT1(:,:,i)=[
            tem(i,6) tem(i,7) tem(i,8) tem(i,3)
            tem(i,9) tem(i,10) tem(i,11) tem(i,4)
            tem(i,12) tem(i,13) tem(i,14) tem(i,5)
            0 0 0 1];
        
        temT2(:,:,i)=[
            tem(i,18) tem(i,19) tem(i,20) tem(i,15)
            tem(i,21) tem(i,22) tem(i,23) tem(i,16)
            tem(i,24) tem(i,25) tem(i,26) tem(i,17)
            0 0 0 1];
    end
    T2_(1:4,:,:)=temT1;
    T2_(5:8,:,:)=temT2;

    % "#"	"t"	"keith_0104_1 X"	"keith_0104_1 Y"	"keith_0104_1 Z"	
    % "keith_0104_1 R11"	"keith_0104_1 R12"	"keith_0104_1 R13"	
    % "keith_0104_1 R21"	"keith_0104_1 R22"	"keith_0104_1 R23"	
    % "keith_0104_1 R31"	"keith_0104_1 R32"	"keith_0104_1 R33"	
    % 
    % "keith_0104_3 X"	"keith_0104_3 Y"	"keith_0104_3 Z"	
    % "keith_0104_3 R11"	"keith_0104_3 R12"	"keith_0104_3 R13"	
    % "keith_0104_3 R21"	"keith_0104_3 R22"	"keith_0104_3 R23"	
    % "keith_0104_3 R31"	"keith_0104_3 R32"	"keith_0104_3 R33"	
    % 
    % "keith_0104_5 X"	"keith_0104_5 Y"	"keith_0104_5 Z"	
    % "keith_0104_5 R11"	"keith_0104_5 R12"	"keith_0104_5 R13"	
    % "keith_0104_5 R21"	"keith_0104_5 R22"	"keith_0104_5 R23"	
    % "keith_0104_5 R31"	"keith_0104_5 R32"	"keith_0104_5 R33"	

    tem=load('data\135_D2.raw');
    block_size=size(tem,1);
    temT1=zeros(4,4,block_size);
    temT2=zeros(4,4,block_size);
    temT3=zeros(4,4,block_size);
    T3_=zeros(12,4,block_size);
    for i = 1 :block_size
        temT1(:,:,i)=[
            tem(i,6) tem(i,7) tem(i,8) tem(i,3)
            tem(i,9) tem(i,10) tem(i,11) tem(i,4)
            tem(i,12) tem(i,13) tem(i,14) tem(i,5)
            0 0 0 1];
        temT2(:,:,i)=[
            tem(i,18) tem(i,19) tem(i,20) tem(i,15)
            tem(i,21) tem(i,22) tem(i,23) tem(i,16)
            tem(i,24) tem(i,25) tem(i,26) tem(i,17)
            0 0 0 1];
        temT3(:,:,i)=[
            tem(i,30) tem(i,31) tem(i,32) tem(i,27)
            tem(i,33) tem(i,34) tem(i,35) tem(i,28)
            tem(i,36) tem(i,37) tem(i,38) tem(i,29)
            0 0 0 1];        
    end
    T3_(1:4,:,:)=temT1;
    T3_(5:8,:,:)=temT2;
    T3_(9:12,:,:)=temT3;
end

T1=T1_;
T2=T2_;
T3=T3_;
end