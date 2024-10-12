function [Pm, Hd]=plotCoord(T,length,flag,linestyle)
if nargin==3
    linestyle='--';
end
if nargin==2
    flag=1;
    linestyle='--';
end
if nargin==1
    length=1;
    flag = 1;
    linestyle='--';
end
    arrowLen=length*10;
    lineWidth=length;
    R=T(1:3,1:3);
    P=T(1:3,4);
    px = R*[arrowLen 0 0]'+P;
    py = R*[0 arrowLen 0]'+P;
    pz = R*[0 0 arrowLen]'+P;
    Hd=0;
    if(flag==2)
        color = 'r';
    elseif(flag == 3)
        color = 'g';
    elseif(flag == 4)
        color = 'b';
    elseif(flag == 5)
        color = 'c';
    elseif(flag>0)
        flag = 1;
    end
    if(flag>0)
        if(flag == 1)
            h1 = line([P(1) px(1)],[P(2) px(2)],[P(3) px(3)],'Color','r','LineStyle',linestyle,'LineWidth',lineWidth);
            h2 = line([P(1) py(1)],[P(2) py(2)],[P(3) py(3)], 'Color','g','LineStyle',linestyle,'LineWidth',lineWidth);
            h3 = line([P(1) pz(1)],[P(2) pz(2)],[P(3) pz(3)], 'Color','b','LineStyle',linestyle,'LineWidth',lineWidth);
        else
            h1 = line([P(1) px(1)],[P(2) px(2)],[P(3) px(3)],'Color',color,'LineStyle','-','LineWidth',lineWidth);
            h2 = line([P(1) py(1)],[P(2) py(2)],[P(3) py(3)], 'Color',color,'LineStyle','-','LineWidth',lineWidth);
            h3 = line([P(1) pz(1)],[P(2) pz(2)],[P(3) pz(3)], 'Color',color,'LineStyle','-','LineWidth',lineWidth);
        end

        plot3(P(1),P(2),P(3),'c*');
        Hd = [h1 h2 h3];
    end
    Pm = [P px py pz];
    
end