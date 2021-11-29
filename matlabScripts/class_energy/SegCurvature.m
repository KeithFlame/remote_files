%
%
%
%
%%
classdef SegCurvature
    properties
        length;
        arm_type;
    end
    properties(Access='private')
        clearance=1e-3;
        MP;
        L0;
    end
    properties(Dependent)
        
    end
    methods
        function sc=SegCurvature(length,arm_type)
            if (nargin<1)
                sc.arm_type=0;
                sc.length=50e-3;
            elseif(nargin<2)
                sc.arm_type=0;
                sc.length=length;
            else
                sc.arm_type=arm_type;
                sc.length=length;
            end
            
        end
    end % constructive function

    methods
        function obj = set.clearance(obj,clearance)
            if(clearance>-1e-9)
                obj.clearance=clearance;
            end
        end % set clearance


    end

    methods  % get
        function obj = get.MP(obj)
            if(obj.arm_type==0)
                obj.MP=getMP(obj.L0);
            end
        end % get MP

        function obj=get.L0(obj)
            [~, L1, ~, Lr, ~, L2, ~, ~,~,~,~]=getToolArmStructureParameter;
            temL=obj.length-L1-Lr-L2;
            if(temL<0)
                obj.L0=0;
            else
                obj.L0=temL;
            end
        end % get L0
    end
end