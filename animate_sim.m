function animate_sim(robot_model_1,ee_att_1,robot_model_2,ee_att_2,high_quality,robot_1_on,robot_2_on,base_on,out)

    if robot_model_1 == 2 || robot_model_1 == 3 || robot_model_2 == 2 || robot_model_2 == 3
        if high_quality == 1
            addpath('D:\Cloud Storage\MATLAB Drive\cad_files\rx160_series\visual');
            time_factor = 10;    
        else
            addpath('D:\Cloud Storage\MATLAB Drive\cad_files\rx160_series\collision');
            time_factor = 5;    
        end
    end

    % Import data from Simulink
    q_posFbk_1 = out.jointPosFbk.Data';
    q_posDes_1 = out.jointPosDes.Data';
    q_posDes_2 = out.jointPosDes2.Data';
    d_c = out.eePosCompress.Data';
    time = 1:length(q_posFbk_1);

    % Robot parameters
    n = 6;

    if ee_att_1 > 0 || ee_att_2 > 0 
        addpath('D:\Cloud Storage\MATLAB Drive\cad_files');
    end

    % zoom = 0;
    
    % Load CAD files
    if robot_1_on == 1
        temp = load('base_link.mat');  ms_1.s0 = temp.modelstruct;
        temp = load('link_1.mat');     ms_1.s1 = temp.modelstruct;
        temp = load('link_2.mat');     ms_1.s2 = temp.modelstruct;
        temp = load('link_3.mat');     ms_1.s3 = temp.modelstruct;
        if robot_model_1 ~= 3
            temp = load('link_4.mat');
        else
            temp = load('link_4l.mat');
        end
        ms_1.s4 = temp.modelstruct;
        temp = load('link_5.mat');     ms_1.s5 = temp.modelstruct;
        temp = load('link_6.mat');     ms_1.s6 = temp.modelstruct;
    end

    if robot_2_on == 1
        temp = load('base_link.mat');  ms_2.s0 = temp.modelstruct;
        temp = load('link_1.mat');     ms_2.s1 = temp.modelstruct;
        temp = load('link_2.mat');     ms_2.s2 = temp.modelstruct;
        temp = load('link_3.mat');     ms_2.s3 = temp.modelstruct;
        if robot_model_2 ~= 3
            temp = load('link_4.mat');
        else
            temp = load('link_4l.mat');
        end
        ms_2.s4 = temp.modelstruct;
        temp = load('link_5.mat');     ms_2.s5 = temp.modelstruct;
        temp = load('link_6.mat');     ms_2.s6 = temp.modelstruct;
    end

    if robot_1_on == 1 || robot_2_on == 1
        if base_on == 1, temp = load('base_block.mat');  ms.sB = temp.modelstruct; end
        if ee_att_1 > 0 || ee_att_2 > 0, temp = load('ati_delta.mat');  ms.sS = temp.modelstruct; end
        if ee_att_1 == 2 || ee_att_1 == 5 || ee_att_2 == 2 || ee_att_2 == 5
            temp = load('gripper.mat');  ms.sG = temp.modelstruct;
        end
        if ee_att_1 == 3 || ee_att_2 == 3
            temp = load('adaptor_holder_new2.mat');  ms.sTa = temp.modelstruct;
            temp = load('rod_sphere_new2.mat');  ms.sTb = temp.modelstruct;
        end
        if ee_att_1 == 4 || ee_att_1 == 5 || ee_att_2 == 4 || ee_att_2 == 5
            temp = load('object2.mat');  ms.sO = temp.modelstruct;
        end
    end
        
    clear temp

    % Generate animation figure
    animFig = figure('Name','Animation Plot','WindowState', 'maximized');
    animPlot = axes('Parent',animFig);
    light                               % add a default light
    view(3)                             % Isometric view

%         view([-240 8.5]);
%         view([-180 90]);
    %     view([-1.827799224162795e+02,14.052024407654459]);
    %     view([-2.079759829144727e+02,45.863193111051032]);
%         CameraPosition=([15 8 4]);

    xlabel(animPlot,'x')
    ylabel(animPlot,'y')
    zlabel(animPlot,'z')
    grid(animPlot,'on')

    if robot_1_on == 1
        Pobj_1 = modelProperties(animPlot,ms,ms_1,base_on,ee_att_1);
        
        [a_i_1,alpha_i_1,d_i_1,thetaPlus_i_1] = kinematicParametersOrigin(robot_model_1);

        if robot_model_1 == 2
            sensor_angle_1 = pi/4;
        else
            sensor_angle_1 = 0;
        end
    end

    if robot_2_on == 1
        Pobj_2 = modelProperties(animPlot,ms,ms_2,base_on,ee_att_2);
        
        [a_i_2,alpha_i_2,d_i_2,thetaPlus_i_2] = kinematicParametersOrigin(robot_model_2);

        if robot_model_2 == 2
            sensor_angle_2 = pi/4;
        else
            sensor_angle_2 = 0;
        end
    end

    limDim = zeros(3,2);

    sensor_length = 0.0333;
    gripper_length = 0.150;
    tool_lengthA = 0.100;
    tool_lengthB = 0.081;
    object_length = 0.02;
    
    if robot_2_on == 1
        maxX = 0.3;
        minX = -0.8;
        maxY = 2.75;
        minY = -0.3;
        maxZ = 0.0;
    else
        maxX = 0.3;
        minX = -0.3;
        maxY = 0.3;
        minY = -0.3;
        maxZ = 0.0;
    end
    if base_on == 1, minZ = -0.6; else, minZ = 0.0; end
    
    if robot_1_on == 1
        switch ee_att_1
            case 1
                m = n+2;
            case 2
                m = n+3;
            case 3
                m = n+4;
            case 4
                m = n+3;
            case 5
                m = n+4;
            otherwise
                m = n+1;
        end
        for j = 1:m
            for t = 1:time_factor:length(time)
                
                TI_0_1	= eye(4);

                TFbkI_h_1 = getTransMatrix(TI_0_1,a_i_1,alpha_i_1,d_i_1,thetaPlus_i_1+q_posFbk_1(:,t));
                
                if j > n+1
                    TFbkI_h_1(:,:,n+2) = TFbkI_h_1(:,:,n+1)*trnZ(sensor_length);
                    if j > n+2
                        if ee_att_1 == 2 || ee_att_1 == 5
                            TFbkI_h_1(:,:,n+3) = TFbkI_h_1(:,:,n+2)*trnZ(gripper_length);
                        elseif ee_att_1 == 3
                            TFbkI_h_1(:,:,n+3) = TFbkI_h_1(:,:,n+2)*trnZ(tool_lengthA);
                        else
                            TFbkI_h_1(:,:,n+3) = TFbkI_h_1(:,:,n+2)*trnZ(object_length);
                        end
                        if j > n+3
                            if ee_att_1 == 3
                                TFbkI_h_1(:,:,n+4) = TFbkI_h_1(:,:,n+3)*trnZ(tool_lengthB);
                            else
                                TFbkI_h_1(:,:,n+4) = TFbkI_h_1(:,:,n+3)*trnZ(object_length);
                            end
                        end
                    end
                end

                if TFbkI_h_1(1,4,j) > maxX, maxX = TFbkI_h_1(1,4,j); end
                if TFbkI_h_1(1,4,j) < minX, minX = TFbkI_h_1(1,4,j); end
                if robot_2_on == 0
                    if TFbkI_h_1(2,4,j) > maxY, maxY = TFbkI_h_1(2,4,j); end
                end
                if TFbkI_h_1(2,4,j) < minY, minY = TFbkI_h_1(2,4,j); end
                if TFbkI_h_1(3,4,j) > maxZ, maxZ = TFbkI_h_1(3,4,j); end
                
            end
        end
    end

    if robot_2_on == 1
        switch ee_att_2
            case 1
                m = n+2;
            case 2
                m = n+3;
            case 3
                m = n+4;
            case 4
                m = n+3;
            case 5
                m = n+4;
            otherwise
                m = n+1;
        end
        for j = 1:m
            for t = 1:time_factor:length(time)
                
                TI_0_2 = [	1 0 0 -0.28
                         	0 1 0 2.43
                          	0 0 1 0
                         	0 0 0 1 ];

                TDesI_h_2 = getTransMatrix(TI_0_2,a_i_2,alpha_i_2,d_i_2,thetaPlus_i_2+q_posDes_2(:,t));
                
                if j > n+1
                    TDesI_h_2(:,:,n+2) = TDesI_h_2(:,:,n+1)*trnZ(sensor_length);
                    if j > n+2
                        if ee_att_2 == 2 || ee_att_2 == 5
                            TDesI_h_2(:,:,n+3) = TDesI_h_2(:,:,n+2)*trnZ(gripper_length);
                        elseif ee_att_2 == 3
                            TDesI_h_2(:,:,n+3) = TDesI_h_2(:,:,n+2)*trnZ(tool_lengthA);
                        else
                            TDesI_h_2(:,:,n+3) = TDesI_h_2(:,:,n+2)*trnZ(object_length);
                        end
                        if j > n+3
                            if ee_att_2 == 3
                                TDesI_h_2(:,:,n+4) = TDesI_h_2(:,:,n+3)*trnZ(tool_lengthB);
                            else
                                TDesI_h_2(:,:,n+4) = TDesI_h_2(:,:,n+3)*trnZ(object_length);
                            end
                        end
                    end
                end

                if TDesI_h_2(1,4,j) > maxX, maxX = TDesI_h_2(1,4,j); end
                if TDesI_h_2(1,4,j) < minX, minX = TDesI_h_2(1,4,j); end
                if TDesI_h_2(2,4,j) > maxY, maxY = TDesI_h_2(2,4,j); end
                if robot_1_on == 0
                    if TDesI_h_2(2,4,j) < minY, minY = TDesI_h_2(2,4,j); end
                end
                if TDesI_h_2(3,4,j) > maxZ, maxZ = TDesI_h_2(3,4,j); end
                
            end
        end
    end

    limDim(1,1) = minX-0.2;
    limDim(1,2) = maxX+0.2;
    limDim(2,1) = minY-0.2;
    limDim(2,2) = maxY+0.2;
    limDim(3,1) = minZ;
    limDim(3,2) = maxZ+0.2;

    if robot_1_on == 1
        PDesI_e_1 = zeros(3,length(time));
        for t = 1:time_factor:length(time)

            TI_0_1	= eye(4);

            TDesI_h_1 = getTransMatrix(TI_0_1,a_i_1,alpha_i_1,d_i_1,thetaPlus_i_1+q_posDes_1(:,t));

            switch ee_att_1
                case 1
                    mountDim = TDesI_h_1(1:3,1:3,end)*[0;0;sensor_length];
                case 2
                    mountDim = TDesI_h_1(1:3,1:3,end)*[0;0;sensor_length+gripper_length];
                case 3
                    mountDim = TDesI_h_1(1:3,1:3,end)*[0;0;sensor_length+tool_lengthA+tool_lengthB];
                case 4
                    mountDim = TDesI_h_1(1:3,1:3,end)*[0;0;sensor_length+object_length];
                case 5
                    mountDim = TDesI_h_1(1:3,1:3,end)*[0;0;sensor_length+gripper_length+object_length];
                otherwise
                    mountDim = zeros(3,1);
            end

            PDesI_e_1(:,t)  = TDesI_h_1(1:3,4,end)+mountDim;

        end
    end

    PFbkI_e_1 = zeros(3,length(time));
    for t = 1:time_factor:length(time)

        hold(animPlot,'on')
        
        if robot_1_on == 1
            
            TI_0_1	= eye(4);

            TFbkI_h_1 = getTransMatrix(TI_0_1,a_i_1,alpha_i_1,d_i_1,thetaPlus_i_1+q_posFbk_1(:,t));

            if ee_att_1 > 0
                TFbkI_h_1(:,:,n+2) = TFbkI_h_1(:,:,n+1)*trnZ(sensor_length);
                if ee_att_1 > 1
                    if ee_att_1 == 2 || ee_att_1 == 5
                        TFbkI_h_1(:,:,n+3) = TFbkI_h_1(:,:,n+2)*trnZ(gripper_length);
                        if ee_att_1 == 5
                            TFbkI_h_1(:,:,n+4) = TFbkI_h_1(:,:,n+3)*trnZ(object_length);
                        end
                    end
                    if ee_att_1 == 3
                        TFbkI_h_1(:,:,n+3) = TFbkI_h_1(:,:,n+2)*trnZ(tool_lengthA);
                        TFbkI_h_1(:,:,n+4) = TFbkI_h_1(:,:,n+3)*trnZ(tool_lengthB+d_c(:,t));
                    end
                    if ee_att_1 == 4
                        TFbkI_h_1(:,:,n+3) = TFbkI_h_1(:,:,n+2)*trnZ(object_length);
                    end
                end
            end

            PFbkI_e_1(:,t)  = TFbkI_h_1(1:3,4,end);

            plot3(animPlot,PDesI_e_1(1,1:time_factor:end),PDesI_e_1(2,1:time_factor:end),PDesI_e_1(3,1:time_factor:end),':b')
            plot3(animPlot,PFbkI_e_1(1,1:time_factor:t),PFbkI_e_1(2,1:time_factor:t),PFbkI_e_1(3,1:time_factor:t),'m')

            if base_on == 1, nvB = [ms.sB.V,ones(size(ms.sB.V(:,1)))]*TFbkI_h_1(:,:,1)'; end
            nv0 = [ms_1.s0.V,ones(size(ms_1.s0.V(:,1)))]*TFbkI_h_1(:,:,1)';
            nv1 = [ms_1.s1.V,ones(size(ms_1.s1.V(:,1)))]*TFbkI_h_1(:,:,2)';
            nv2 = [ms_1.s2.V,ones(size(ms_1.s2.V(:,1)))]*TFbkI_h_1(:,:,3)';
            nv3 = [ms_1.s3.V,ones(size(ms_1.s3.V(:,1)))]*TFbkI_h_1(:,:,4)';
            nv4 = [ms_1.s4.V,ones(size(ms_1.s4.V(:,1)))]*TFbkI_h_1(:,:,5)';
            nv5 = [ms_1.s5.V,ones(size(ms_1.s5.V(:,1)))]*TFbkI_h_1(:,:,6)';
            nv6 = [ms_1.s6.V,ones(size(ms_1.s6.V(:,1)))]*TFbkI_h_1(:,:,7)';
            if ee_att_1 > 0, nvS = [ms.sS.V,ones(size(ms.sS.V(:,1)))]*rotZ(sensor_angle_1)'*TFbkI_h_1(:,:,8)'; end
            if ee_att_1 == 2 || ee_att_1 == 5
                nvG = [ms.sG.V,ones(size(ms.sG.V(:,1)))]*rotZ(sensor_angle_1)'*TFbkI_h_1(:,:,9)';
                if ee_att_1 == 5                    
                    nvO = [ms.sO.V,ones(size(ms.sO.V(:,1)))]*rotZ(sensor_angle_1)'*TFbkI_h_1(:,:,10)';
                end
            end
            if ee_att_1 == 3
                nTa = [ms.sTa.V,ones(size(ms.sTa.V(:,1)))]*rotZ(sensor_angle_1)'*TFbkI_h_1(:,:,9)';
                nTb = [ms.sTb.V,ones(size(ms.sTb.V(:,1)))]*rotZ(sensor_angle_1)'*TFbkI_h_1(:,:,10)';
            end
            if ee_att_1 == 4
                nvO = [ms.sO.V,ones(size(ms.sO.V(:,1)))]*rotZ(sensor_angle_1)'*TFbkI_h_1(:,:,9)';
            end

            if base_on == 1, set(Pobj_1.pB,'Vertices',nvB(:,1:3)), end
            set(Pobj_1.p0,'Vertices',nv0(:,1:3))
            set(Pobj_1.p1,'Vertices',nv1(:,1:3))
            set(Pobj_1.p2,'Vertices',nv2(:,1:3))
            set(Pobj_1.p3,'Vertices',nv3(:,1:3))
            set(Pobj_1.p4,'Vertices',nv4(:,1:3))
            set(Pobj_1.p5,'Vertices',nv5(:,1:3))
            set(Pobj_1.p6,'Vertices',nv6(:,1:3))
            if ee_att_1 > 0, set(Pobj_1.pS,'Vertices',nvS(:,1:3)), end
            if ee_att_1 == 2 || ee_att_1 == 5
                set(Pobj_1.pG,'Vertices',nvG(:,1:3))
            end
            if ee_att_1 == 3
                set(Pobj_1.pTa,'Vertices',nTa(:,1:3))
                set(Pobj_1.pTb,'Vertices',nTb(:,1:3))
            end
            if ee_att_1 == 4 || ee_att_1 == 5
                set(Pobj_1.pO,'Vertices',nvO(:,1:3))
            end
        end

        if robot_2_on == 1
            
            TI_0_2  = [	1 0 0 -0.28
                      	0 1 0 2.43
                       	0 0 1 0
                       	0 0 0 1 ];

            TDesI_h_2 = getTransMatrix(TI_0_2,a_i_2,alpha_i_2,d_i_2,thetaPlus_i_2+q_posDes_2(:,t));

            if ee_att_2 > 0
                TDesI_h_2(:,:,n+2) = TDesI_h_2(:,:,n+1)*trnZ(sensor_length);
                if ee_att_2 > 1
                    if ee_att_2 == 2 || ee_att_2 == 5
                        TDesI_h_2(:,:,n+3) = TDesI_h_2(:,:,n+2)*trnZ(gripper_length);
                        if ee_att_2 == 5
                            TDesI_h_2(:,:,n+4) = TDesI_h_2(:,:,n+3)*trnZ(object_length);
                        end
                    end
                    if ee_att_2 == 3
                        TDesI_h_2(:,:,n+3) = TDesI_h_2(:,:,n+2)*trnZ(tool_lengthA);
                        TDesI_h_2(:,:,n+4) = TDesI_h_2(:,:,n+3)*trnZ(tool_lengthB);
                    end
                    if ee_att_2 == 4
                        TDesI_h_2(:,:,n+3) = TDesI_h_2(:,:,n+2)*trnZ(object_length);
                    end
                end
            end

            if base_on == 1, nvB = [ms.sB.V,ones(size(ms.sB.V(:,1)))]*TDesI_h_2(:,:,1)'; end
            nv0 = [ms_2.s0.V,ones(size(ms_2.s0.V(:,1)))]*TDesI_h_2(:,:,1)';
            nv1 = [ms_2.s1.V,ones(size(ms_2.s1.V(:,1)))]*TDesI_h_2(:,:,2)';
            nv2 = [ms_2.s2.V,ones(size(ms_2.s2.V(:,1)))]*TDesI_h_2(:,:,3)';
            nv3 = [ms_2.s3.V,ones(size(ms_2.s3.V(:,1)))]*TDesI_h_2(:,:,4)';
            nv4 = [ms_2.s4.V,ones(size(ms_2.s4.V(:,1)))]*TDesI_h_2(:,:,5)';
            nv5 = [ms_2.s5.V,ones(size(ms_2.s5.V(:,1)))]*TDesI_h_2(:,:,6)';
            nv6 = [ms_2.s6.V,ones(size(ms_2.s6.V(:,1)))]*TDesI_h_2(:,:,7)';
            if ee_att_2 > 0, nvS = [ms.sS.V,ones(size(ms.sS.V(:,1)))]*rotZ(sensor_angle_2)'*TDesI_h_2(:,:,8)'; end
            if ee_att_2 == 2 || ee_att_2 == 5
                nvG = [ms.sG.V,ones(size(ms.sG.V(:,1)))]*rotZ(sensor_angle_2)'*TDesI_h_2(:,:,9)';
                if ee_att_2 == 5                    
                    nvO = [ms.sO.V,ones(size(ms.sO.V(:,1)))]*rotZ(sensor_angle_2)'*TDesI_h_2(:,:,10)';
                end
            end
            if ee_att_2 == 3
                nTa = [ms.sTa.V,ones(size(ms.sTa.V(:,1)))]*rotZ(sensor_angle_2)'*TDesI_h_2(:,:,9)';
                nTb = [ms.sTb.V,ones(size(ms.sTb.V(:,1)))]*rotZ(sensor_angle_2)'*TDesI_h_2(:,:,10)';
            end
            if ee_att_2 == 4
                nvO = [ms.sO.V,ones(size(ms.sO.V(:,1)))]*rotZ(sensor_angle_2)'*TDesI_h_2(:,:,9)';
            end

            if base_on == 1, set(Pobj_2.pB,'Vertices',nvB(:,1:3)), end
            set(Pobj_2.p0,'Vertices',nv0(:,1:3))
            set(Pobj_2.p1,'Vertices',nv1(:,1:3))
            set(Pobj_2.p2,'Vertices',nv2(:,1:3))
            set(Pobj_2.p3,'Vertices',nv3(:,1:3))
            set(Pobj_2.p4,'Vertices',nv4(:,1:3))
            set(Pobj_2.p5,'Vertices',nv5(:,1:3))
            set(Pobj_2.p6,'Vertices',nv6(:,1:3))
            if ee_att_2 > 0, set(Pobj_2.pS,'Vertices',nvS(:,1:3)), end
            if ee_att_2 == 2 || ee_att_2 == 5
                set(Pobj_2.pG,'Vertices',nvG(:,1:3))
            end
            if ee_att_2 == 3
                set(Pobj_2.pTa,'Vertices',nTa(:,1:3))
                set(Pobj_2.p8b,'Vertices',nTb(:,1:3))
            end
            if ee_att_2 == 4 || ee_att_2 == 5, set(Pobj_2.pO,'Vertices',nvO(:,1:3)), end
        end

        hold(animPlot,'off')

        set(animPlot,	'Projection','perspective', ...
                        'PlotBoxAspectRatio',[1 1 1], ...
                        'DataAspectRatio',[1 1 1], ...
                        'XLim',[limDim(1,1) limDim(1,2)], ...
                        'YLim',[limDim(2,1) limDim(2,2)], ...
                        'ZLim',[limDim(3,1) limDim(3,2)]);

        drawnow;
    end

    function rot_z = rotZ(rad)        
        rot_z = [	cos(rad)	-sin(rad)	0   0
                    sin(rad)	cos(rad)	0   0
                    0           0           1   0  
                    0           0           0   1];
    end

    function trn_z = trnZ(len)      
        trn_z = [ 1 0 0 0;
                 0 1 0 0
                 0 0 1 len
                 0 0 0 1 ];
    end

    function Pobj = modelProperties(animPlot,ms,ms_N,base_on,ee_att)

        % CAD properties
        if base_on == 1
            Pobj.pB = patch(animPlot,'Faces', ms.sB.F, 'Vertices', ms.sB.V);
            set(Pobj.pB, 'facec', 'flat');              % Set the face color flat
            set(Pobj.pB, 'FaceColor', ms.sB.C);         % Set the color (from file)
            set(Pobj.pB, 'EdgeColor', 'none');          % Set the edge color
        end
        Pobj.p0 = patch(animPlot,'Faces', ms_N.s0.F, 'Vertices', ms_N.s0.V);
        set(Pobj.p0, 'facec', 'flat');                  % Set the face color flat
        set(Pobj.p0, 'FaceColor', ms_N.s0.C);           % Set the color (from file)
        set(Pobj.p0, 'EdgeColor', 'none');              % Set the edge color
        Pobj.p1 = patch(animPlot,'Faces', ms_N.s1.F, 'Vertices', ms_N.s1.V);
        set(Pobj.p1, 'facec', 'flat');                  % Set the face color flat
        set(Pobj.p1, 'FaceColor', ms_N.s1.C);           % Set the color (from file)
        set(Pobj.p1, 'EdgeColor', 'none');              % Set the edge color
        Pobj.p2 = patch(animPlot,'Faces', ms_N.s2.F, 'Vertices', ms_N.s2.V);
        set(Pobj.p2, 'facec', 'flat');                  % Set the face color flat
        set(Pobj.p2, 'FaceColor', ms_N.s2.C);           % Set the color (from file)
        set(Pobj.p2, 'EdgeColor', 'none');              % Set the edge color
        Pobj.p3 = patch(animPlot,'Faces', ms_N.s3.F, 'Vertices', ms_N.s3.V);
        set(Pobj.p3, 'facec', 'flat');                  % Set the face color flat
        set(Pobj.p3, 'FaceColor', ms_N.s3.C);           % Set the color (from file)
        set(Pobj.p3, 'EdgeColor', 'none');              % Set the edge color
        Pobj.p4 = patch(animPlot,'Faces', ms_N.s4.F, 'Vertices', ms_N.s4.V);
        set(Pobj.p4, 'facec', 'flat');                  % Set the face color flat
        set(Pobj.p4, 'FaceColor', ms_N.s4.C);           % Set the color (from file)
        set(Pobj.p4, 'EdgeColor', 'none');              % Set the edge color
        Pobj.p5 = patch(animPlot,'Faces', ms_N.s5.F, 'Vertices', ms_N.s5.V);
        set(Pobj.p5, 'facec', 'flat');                  % Set the face color flat
        set(Pobj.p5, 'FaceColor', ms_N.s5.C);           % Set the color (from file)
        set(Pobj.p5, 'EdgeColor', 'none');              % Set the edge color
        Pobj.p6 = patch(animPlot,'Faces', ms_N.s6.F, 'Vertices', ms_N.s6.V);
        set(Pobj.p6, 'facec', 'flat');                  % Set the face color flat
        set(Pobj.p6, 'FaceColor', ms_N.s6.C);           % Set the color (from file)
        set(Pobj.p6, 'EdgeColor', 'none');              % Set the edge color        
        if ee_att > 0
            Pobj.pS = patch(animPlot,'Faces', ms.sS.F, 'Vertices', ms.sS.V);
            set(Pobj.pS, 'facec', 'flat');              % Set the face color flat
            set(Pobj.pS, 'FaceVertexCData', ms.sS.C); 	% Set the color (from file)
            set(Pobj.pS, 'EdgeColor', 'none');          % Set the edge color
        end
        if ee_att == 2 || ee_att == 5
            Pobj.pG = patch(animPlot,'Faces', ms.sG.F, 'Vertices', ms.sG.V);
            set(Pobj.pG, 'facec', 'flat');              % Set the face color flat
            set(Pobj.pG, 'FaceColor', ms.sG.C);         % Set the color (from file)
            set(Pobj.pG, 'EdgeColor', 'none');          % Set the edge color
        end
        if ee_att == 3
            Pobj.pTa = patch(animPlot,'Faces', ms.sTa.F, 'Vertices', ms.sTa.V);
            set(Pobj.pTa, 'facec', 'flat');             % Set the face color flat
            set(Pobj.pTa, 'FaceColor', ms.sTa.C);       % Set the color (from file)
            set(Pobj.pTa, 'EdgeColor', 'none');         % Set the edge color
            
            Pobj.pTb = patch(animPlot,'Faces', ms.sTb.F, 'Vertices', ms.sTb.V);
            set(Pobj.pTb, 'facec', 'flat');             % Set the face color flat
            set(Pobj.pTb, 'FaceColor', ms.sTb.C);       % Set the color (from file)
            set(Pobj.pTb, 'EdgeColor', 'none');         % Set the edge color
        end
        if ee_att == 4 || ee_att == 5
            Pobj.pO = patch(animPlot,'Faces', ms.sO.F, 'Vertices', ms.sO.V);
            set(Pobj.pO, 'facec', 'flat');              % Set the face color flat
            set(Pobj.pO, 'FaceColor', ms.sO.C);         % Set the color (from file)
            set(Pobj.pO, 'EdgeColor', 'none');          % Set the edge color
        end
        
    end

    function TI_h = getTransMatrix(TI_0,a_i,alpha_i,d_i,theta_i)

        k = length(theta_i);

        TI_h = zeros(4,4,k+1);

        TI_h(:,:,1) = TI_0;

        for i = 1:k        
            Rot_x_h = Rot_x(alpha_i(i));
            Trn_x_h	= Trn_x(a_i(i));
            Rot_z_i = Rot_z(theta_i(i));
            Trn_z_i = Trn_z(d_i(i));

            TI_h(:,:,i+1) = TI_h(:,:,i) * Rot_x_h * Trn_x_h * Rot_z_i * Trn_z_i;
        end

        function output = Rot_x(input)
            output = [	1	0           0           0
                        0	cos(input)  -sin(input)	0
                        0	sin(input)	cos(input)	0
                        0	0           0           1 ];
        end

        function output = Rot_z(input)
            output = [  cos(input)	-sin(input)	0	0
                        sin(input)	 cos(input)	0	0
                        0            0          1	0
                        0            0          0	1 ];
        end

        function output = Trn_x(input)
            output = [  1 0 0 input
                        0 1 0 0
                        0 0 1 0
                        0 0 0 1     ];
        end

        function output = Trn_z(input)
            output = [  1 0 0 0
                        0 1 0 0
                        0 0 1 input
                        0 0 0 1     ];
        end
    end

    function vert = vertice_topZ_gen(O,L)

    vert = [O(1)-L(1)/2 O(2)-L(2)/2 O(3)-L(3); ...
            O(1)-L(1)/2 O(2)+L(2)/2 O(3)-L(3); ...
            O(1)+L(1)/2 O(2)+L(2)/2 O(3)-L(3); ...
            O(1)+L(1)/2 O(2)-L(2)/2 O(3)-L(3); ...
            O(1)-L(1)/2 O(2)-L(2)/2 O(3); ...
            O(1)-L(1)/2 O(2)+L(2)/2 O(3); ...
            O(1)+L(1)/2 O(2)+L(2)/2 O(3); ...
            O(1)+L(1)/2 O(2)-L(2)/2 O(3)];

    end

end