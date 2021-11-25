Rot = expm(S([0 0 pi/4]'));
ax = gca;
N=length(ax.Children);
for i=1:N
    test=ax.Children(i).XData;
    testsize = size(test);
    
    if(testsize(1)>1)%patch
        Data = ax.Children(i).Vertices;
        Data=Data';
        Data = Rot*Data;
        ax.Children(i).Vertices = Data';
    else
        Data = [ax.Children(i).XData;ax.Children(i).YData;ax.Children(i).ZData];
        Data = Rot*Data;
        ax.Children(i).XData = Data(1,:);
        ax.Children(i).YData = Data(2,:);
        ax.Children(i).ZData = Data(3,:);
    end
    
end    