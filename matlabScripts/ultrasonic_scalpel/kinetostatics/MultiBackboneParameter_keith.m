classdef MultiBackboneParameter_keith
    %MULTIBACKBONEPARAMETER_KEITH 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        
        % material property
        E                           % Elastic module
        mu                          % Rod Poisson rate
        G                           % shear modulus

        % structural properties
        d1                          % Rod diameter seg1
        d2                          % Rod diameter seg2
        Lstem                       % Robot stem length
        L1                          % Robot seg1 length
        L2                          % Robot seg2 length
        Lr                          % Robot rigid seg length
        Lg                          % Robot gipper length
        Ldo                         % Deformable object length
        A1                          % crosssection area for rod in seg1
        A2                          % crosssection area for rod in seg2
        I1                          % second moment of inertia for seg1
        I2                          % second moment of inertia for seg2
        Ke1                         % stiffness matrix for seg1
        Ke2                         % stiffness matrix for seg2
        Kb1                         % stiffness matrix for seg1
        Kb2                         % stiffness matrix for seg2   

        % fabrication properties
        rho1                        % Rod pitch circle radius seg1
        rho2                        % Rod pithc circle radius seg2        
        delta_t1                    % angle between first rod and Y-axis for seg1 
        delta_t2                    % angle between first rod and Y-axis for seg2
        K1                          % stiffness coefficient for seg1
        K2                          % stiffness coefficient for seg2
        zeta                        % bending performance of the stem
        gamma1                      % offset for the rotation DoF
        gamma2                      % offset for rotation in the two segments
        gamma3                      % offset for rotation in 2nd seg and gripper

        % intermediate variable
        r11                         % position for first rod in XY plane for seg1
        r12                         % position for second rod in XY plane for seg1
        r21                         % position for first rod in XY plane for seg2
        r22                         % position for second rod in XY plane for seg2
        Q1                          % all rod's positions in XY plane for seg1
        Q2                          % all rod's positions in XY plane for seg2
        discrete_element            % discrete element

        
        % service properties
        L1o                         % seg1 length out of the trocar
        Ls                          % stem length out of trocar

        % several Matrix
        dGamma
        Ell
        THETA
        Ka
        Gc
        % IAUS Specifications
        b_P_port
        L_gravity
        b_G

        g_P_tip
        g_R_tip
        
    end
    
    methods
        function obj = MultiBackboneParameter_keith(MP,SP,FP)
            %MULTIBACKBONEPARAMETER_KEITH 构造此类的实例
            %   此处显示详细说明
            if(nargin == 0)
                MP = [40e9 0.33];
                SP = [0.95e-3 0.4e-3 0.6 0.1 0.01 0.02 0.015];
                FP = [2.5e-3 2.7e-3 0 -pi/4 0 0 0.2 0 0 0];
            end
            obj.E=MP(1);                          
            obj.mu=MP(2);
            obj.G = obj.E / 2 / (1 + obj.mu);

            obj.d1=SP(1);                      
            obj.d2=SP(2);  
            obj.Lstem = SP(3);
            obj.L1 = SP(4);
            obj.Lr = SP(5);
            obj.L2 = SP(6);
            obj.Lg = SP(7);
            obj.Ldo = SP(6);
            obj.A1 = pi * obj.d1 ^ 2 / 4;
            obj.A2 = pi * obj.d2 ^ 2;
            obj.I1 = pi * obj.d1 ^ 4 / 64;
            obj.I2 = pi * obj.d2 ^ 4 / 64;
            obj.Ke1 = diag([obj.G*obj.A1 obj.G*obj.A1 obj.A1*obj.E]);
            obj.Ke2 = diag([obj.G*obj.A2 obj.G*obj.A2 obj.A2*obj.E]);
            obj.Kb1 = diag([obj.E*obj.I1 obj.E*obj.I1 2*obj.I1*obj.G]);
            obj.Kb2 = diag([obj.E*obj.I2 obj.E*obj.I2 2*obj.I2*obj.G]);         
            
            obj.rho1 = FP(1);                     
            obj.rho2 = FP(2);
            obj.delta_t1 = FP(3);
            obj.delta_t2 = FP(4);
            obj.K1 = FP(5);
            obj.K2 = FP(6);
            obj.zeta = FP(7);
            obj.gamma1 = FP(8);
            obj.gamma2 = FP(9);
            obj.gamma3 = FP(10);
                                    
            obj.r11=[cos(obj.delta_t1) sin(obj.delta_t1) 0]'*obj.rho1;
            obj.r12=[cos(obj.delta_t1 + pi/2) sin(obj.delta_t1 + pi/2) 0]'*obj.rho1;
            obj.r21=[cos(obj.delta_t2) sin(obj.delta_t2) 0]'*obj.rho2;
            obj.r22=[cos(-obj.delta_t2) sin(-obj.delta_t2) 0]'*obj.rho2;
    
            e3 = [0 0 1]';
            obj.Q1=[skewMatrix_keith(obj.r11)*e3 skewMatrix_keith(obj.r12)*e3 ...
                skewMatrix_keith(obj.r21)*e3 skewMatrix_keith(obj.r22)*e3];
            obj.Q2=[skewMatrix_keith(obj.r21)*e3 skewMatrix_keith(obj.r22)*e3];
            obj.discrete_element = 1e-3;
    
            obj = refreshLso(obj,40);

            obj.dGamma = zeros(4,6);
            obj.Ell=zeros(4,4);
            obj.THETA=zeros(6,4);
            obj.Ka=zeros(6,6);
            
                    
            obj.b_P_port = [60;0;0]*1e-3;
            obj.L_gravity = 200e-3;
            obj.b_G = [2;0;0];

            obj.g_P_tip = [0;0;37]*1e-3;
            obj.g_R_tip = RotmAxisY(-10/180*pi);
        
        end
        function obj = resetCalibrationPara(obj, SL)
            % name:  L1 Lr L2 Lg zeta K1  K2   gamma1 Lstem gamma3
            % SL = [100 10 20 15 0.2   5  0.6   pi/40   600 3];
            len_SL = length(SL);
            try
                assert(len_SL==10);
            catch
                warning(['Problem occurred because length of SL is %d and LESS THAN 10.\n',...
                    'And 10 calibrated variable are L1, Lr, L2, Lg, zeta, K1, K2, ',...
                    'gamma1, Lstem, and gamma3, respectively.\n'],len_SL);
                return;
            end 
            obj.L1 = SL(1) * 1e-3;
            obj.Lr = SL(2) * 1e-3;
            obj.L2 = SL(3) * 1e-3;
            obj.Lg = SL(4) * 1e-3;
            obj.zeta = SL(5);
            obj.K1 = SL(6);
            obj.K2 = SL(7);
            obj.gamma1 = SL(8);
            obj.Lstem = SL(9) * 1e-3;
            obj.gamma3 = SL(10);
        end
        function obj = resetL1r2g(obj,SL)
            obj.L1 = SL(1) * 1e-3;
            obj.Lr = SL(2) * 1e-3;
            obj.L2 = SL(3) * 1e-3;
            obj.Lg = SL(4) * 1e-3;            
        end
        function obj = resetK12(obj,SL)
            obj.K1 = SL(1);
            obj.K2 = SL(2);           
        end
        function obj = resetK1(obj,SL)
            obj.K1 = SL;
        end
        function obj = resetZeta(obj,SL)
            obj.zeta = SL;
        end
        function obj = resetLg(obj,SL)
            obj.Lg = SL;
        end
        function obj = resetGamma1(obj,SL)
            obj.gamma1 = SL;
        end
        function obj = resetLstem(obj,SL)
            obj.Lstem = SL;
        end
        function obj = setLdo(obj,Ldo)
            obj.Ldo = Ldo;
        end
        function obj = calcMatrix(obj)         
            obj.dGamma(1:4,1:3)=(obj.L1o+obj.Ls*obj.zeta)*obj.Q1';
            obj.dGamma(3:4,4:6)=obj.L2*obj.Q2';
            
            obj.Ell(1:2,1:2)=diag([1 1])*(obj.L1+obj.Lstem);
            obj.Ell(3:4,3:4)=diag([1 1])*(obj.L2+obj.Lr+obj.L1+obj.Lstem);
            
            obj.THETA(1:3,1:4)=-obj.A1*2*obj.E*obj.Q1;
            obj.THETA(1:3,3:4)=-obj.A1*obj.E*obj.Q2*0;
            obj.THETA(4:6,3:4)=-obj.A2*2*obj.E*obj.Q2;
            
            obj.Ka(1:3,1:3)=4*obj.Kb1+16*obj.Kb2+obj.K1*obj.Kb1;
            obj.Ka(1:3,4:6)=-16*obj.Kb2-obj.K2*obj.Kb1;
            obj.Ka(4:6,4:6)=16*obj.Kb2+obj.K2*obj.Kb1;
            
            obj.Gc=obj.dGamma-obj.Ell*pinv(obj.THETA)*obj.Ka;
        end
        function obj = refreshLso(obj,l)
            l = l * 1e-3;
            if(l>obj.L1)
                obj.Ls = l-obj.L1;
                obj.L1o = obj.L1;
            else
                obj.Ls = 0;
                obj.L1o = l;
            end
            obj = calcMatrix(obj); 
        end
    end
end

