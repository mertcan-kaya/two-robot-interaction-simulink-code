function [m_i,ri_i_ci,Ii_i] = staticInertialParameters(robot_model,ee_att)

    % These parameters are taken from https://github.com/ros-industrial/staubli

    [m_6,r6_6_c6] = getEEdynPara(robot_model,ee_att);
    
    if robot_model == 2
        m_i = [45.357763; 51.266495; 19.754401; 15.287896; 0.548088; m_6];
    elseif robot_model == 3
        m_i = [45.357763; 51.266495; 19.754401; 27.640019; 0.548088; m_6];
    else
        m_i = zeros(6,1);
    end
    
    ri_i_ci = zeros(3,1,6);

    ri_i_ci(:,:,1) = [0.085479;-0.002547;-0.040488];
    ri_i_ci(:,:,2) = [-0.000002;0.264164;0.347704];
    ri_i_ci(:,:,3) = [-0.000255;0.016478;-0.003819];
    if robot_model == 2
        ri_i_ci(:,:,4) = [-0.015515;0.000164;0.340348];
    else     
        ri_i_ci(:,:,4) = [-0.009602;0.000028;0.472641];
    end
    ri_i_ci(:,:,5) = [0.000000;-0.000347;0.023671];
    ri_i_ci(:,:,6) = r6_6_c6;

    % Inertia tensors
    Ii_i = zeros(3,3,6);

    Ii_i(:,:,1) = [	0.839141    0.022637     0.162457
                    0.022637    1.030146     0.027416
                    0.162457    0.027416     1.064698];

    Ii_i(:,:,2) = [ 5.272800    0.000010    0.000017
                    0.000010    5.536809    -0.008261
                    0.000017    -0.008261   0.486191];

    Ii_i(:,:,3) = [	0.249388    -0.004901   0.004825 
                    -0.004901   0.211780    0.000574 
                    0.004825    0.000574    0.238488];
    
    if robot_model == 2
        Ii_i(:,:,4) = [ 0.331593    0.000055    -0.010688
        	            0.000055    0.320895    0.000058
                        -0.010688   0.000058    0.086557];
    else
        Ii_i(:,:,4) = [ 1.378080    0.000062    -0.042670
        	            0.000062    1.366754    0.000069
                        -0.042670   0.000069    0.170331];
    end
    
    Ii_i(:,:,5) = [ 0.000876    0.000000    0.000000
        	        0.000000    0.000889    0.000005
                    0.000000    0.000005    0.000412];

    Ii_i(:,:,6) = [ 0.000011    0.000000    0.000000
        	        0.000000    0.000011    0.000000
                    0.000000    0.000000    0.000021];
    
    function [m_6,r6_6_c6] = getEEdynPara(robot_model,ee_att)
        
        % masses (kg)
        if robot_model == 2 || robot_model == 3
            sixth_link_mass = 0.038484;
            sixth_CoM_6F    = [-0.000245;0.000000;-0.007626];
        else
            sixth_link_mass = 0.0;
            sixth_CoM_6F    = [0.000;0.000;0.000];
        end
    
        outter_sensor_mass = 0.918 - 0.114; % sensor's full mass minus its inner mass (including inner plate mass)
        outter_adapter_mass = 0.253;
        inner_sensor_mass = 0.114;
        gripper_mass = 1.120;
        ball_spring_mass = 1.203;

        if ee_att == 1      % sensor
            tool_mass = inner_sensor_mass;
            m_6 = sixth_link_mass + outter_sensor_mass + tool_mass;
            tool_CoM_sF = [0.0;0.0;-0.005];
        elseif  ee_att == 2	% sensor + gripper
            tool_mass = inner_sensor_mass + outter_adapter_mass + gripper_mass;
            m_6 = sixth_link_mass + outter_sensor_mass + tool_mass;
            tool_CoM_sF = [0.0117;0.0097;0.0622];
        elseif  ee_att == 3	% sensor + ball&spring
            tool_mass = inner_sensor_mass + outter_adapter_mass + ball_spring_mass;
            m_6 = sixth_link_mass + outter_sensor_mass + tool_mass;
            tool_CoM_sF = [0.0076;0.0104;0.0815];
        else                % none
            tool_mass = 0.0;
            m_6 = sixth_link_mass;
            tool_CoM_sF = [0.0;0.0;0.0];
        end
        
        % sensor(outter) center of mass in sixth frame
        sensor_CoM_6F   = [0.000; 0.000;0.0136];

        if robot_model == 2
            sensor_mount_ang = pi/4;
        else
            sensor_mount_ang = 0;
        end

		sensor_length = 0.0333; % m
        
        RotZ = [  cos(-sensor_mount_ang)	-sin(-sensor_mount_ang)	0
                    sin(-sensor_mount_ang)	 cos(-sensor_mount_ang)	0
                    0            0          1];

        tool_CoM_6F = RotZ*tool_CoM_sF+[0;0;sensor_length];

        if ee_att ~= 0      % sensor
            r6_6_c6 = (sixth_link_mass*sixth_CoM_6F+outter_sensor_mass*sensor_CoM_6F+tool_mass*tool_CoM_6F) ...
                /(sixth_link_mass+outter_sensor_mass+tool_mass);
        else
            r6_6_c6 = sixth_CoM_6F;
        end
    
    end

end