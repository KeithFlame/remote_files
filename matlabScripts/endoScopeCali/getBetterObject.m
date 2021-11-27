function [endoQ,endoQ_,endoCur_]=getBetterObject(x)
    x0=setX0;
    gamma3=0.0;
    [T, endoPsi]=getTandConfigV2;
    endoCur=fromPsi2Curvature(endoPsi,x0);
    endoQ=fromCurvature2Movitation(endoCur,x0);
    endoPsi_=fromConfig2Psi(T,x,gamma3);
    endoCur_=fromPsi2Curvature(endoPsi_,x);
    endoQ_=fromCurvature2Movitation(endoCur_,x);
%     for i = size(endoPsi,1):-1:1
%         if(endoPsi_(i,2)-endoPsi(i,2))>1.5
%             endoPsi_(i,:)=[];
%             endoPsi(i,:)=[];
%             endoQ(i,:)=[];
%             endoQ_(i,:)=[];
%             endoCur_(i,:)=[];
%         end
%     end
    for i =size(endoPsi,1):-1:1
        if(norm(endoQ(i,:))==0)
            endoCur(i,:)=[];
            endoPsi_(i,:)=[];
            endoPsi(i,:)=[];
            endoQ(i,:)=[];
            endoQ_(i,:)=[];
            endoCur_(i,:)=[];  
        end
    end
end
    