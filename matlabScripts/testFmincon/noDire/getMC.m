function [T_m, T_c,psi_,sp]=getMC

persistent Tm;
persistent Tc;
persistent Psi_;
persistent SP_;
if(isempty(Tm))
    Tm=load('./59_2/T_m.mat');
end
if(isempty(Tc)||isempty(Psi_)||isempty(SP_))

    psi=load('./59_2/psi.mat');
    SP=load('./59_2/structure_para.mat');
    Psi=psi.Psi';
    SP=SP.LS;
    block_size=max(size(Psi));
    Tc=zeros(4,4,block_size);
    Psi_=Psi;
    SP_=SP;
    for i =1:block_size
        tem_psi=Psi(i,:);
        t=tem_psi(2);
        tem_psi(2)=tem_psi(1)/1000-SP(2)-SP(3)+SP(11);
        tem_psi(1)=t;

        T=fromPsi2Config(tem_psi,SP);
        Tc(:,:,i)=T;
        Psi_(i,:)=tem_psi;
    end
    
    i=0;
    
    
end
T_m=Tm.Tend_m;
T_c=Tc;
psi_=Psi_;
sp=SP_;
end