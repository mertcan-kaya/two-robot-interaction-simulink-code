function d_e = getEElength(ee_att)

    sensor_length  = 0.0333; % m
    gripper_length = 0.150; % m
    ballspr_length = 0.181; % m
    object_length  = 0.020; % m

    if ee_att == 1
        d_e = sensor_length;
    elseif ee_att == 2
        d_e = sensor_length + gripper_length;
    elseif ee_att == 3
        d_e = sensor_length + ballspr_length;
    elseif ee_att == 4
        d_e = sensor_length + object_length;
    elseif ee_att == 5
        d_e = sensor_length + gripper_length + object_length;
    else
        d_e = 0;
    end

end