function tf = computeTrajectoryTime(ip,fp,prcnt,trj_space,trj_profile,robot_model)
    
    if trj_space == 0
        
        if robot_model == 1        
            % joint pos limits (deg to rad)
            posLim = deg2rad([  -180.0,180.0
                                -130.0,147.5
                                -145.0,145.0
                                -270.0,270.0
                                -115.0,140.0
                                -270.0,270.0]);

            % joint abs vel limits (rad/s)
            velLim = deg2rad([400;400;430;540;475;760]);
        else
            % joint pos limits (deg to rad)
            posLim = deg2rad([  -160.0,160.0
                                -137.5,137.5
                                -150.0,150.0
                                -270.0,270.0
                                -105.0,120.0
                                -270.0,270.0]);

            % joint abs vel limits (rad/s)
            velLim = deg2rad([200;200;255;315;360;870]);
        end

        % joint abs acc limits (rad/s^2)
        accLim = [7.9;6.5;10.5;25.2;19.6;41.7];

        v_prcnt = prcnt(1);
        a_prcnt = prcnt(2);

        kv = v_prcnt*velLim;
        ka = a_prcnt*accLim;

        qi = ip;
        qf = fp;
        
        % limit given positions according to the robot model
        for j = 1:6
            if qi(j) < posLim(j,1); qi(j) = posLim(j,1); end
            if qf(j) < posLim(j,1); qf(j) = posLim(j,1); end
            if qi(j) > posLim(j,2); qi(j) = posLim(j,2); end
            if qf(j) > posLim(j,2); qf(j) = posLim(j,2); end
        end

        D = qf-qi;

        switch trj_profile
            case 1
                tf = max(abs(D)./kv);
    %             vMax = abs(D)/tf;
    %             aMax = 0;
            case 2
                tf = max(max(3*abs(D)./(2*kv)),max(sqrt(6*abs(D)./ka)));
    %             vMax = 3*abs(D)/(2*tf);
    %             aMax = 6*abs(D)/tf^2;
            case 3
                tf = max(max(15*abs(D)./(8*kv)),max(sqrt(10*abs(D)./(sqrt(3)*ka))));
    %             vMax = 15*abs(D)/(8*tf);
    %             aMax = 10*abs(D)/(sqrt(3)*tf^2);
            case 4
                tf = max(max(2*abs(D)./kv),max(sqrt(4*abs(D)./ka)));
    %             vMax = 2*abs(D)/tf;
    %             aMax = 4*abs(D)/tf^2;
            otherwise
                tf = 0;
        end

    else
        
        max_tspcVel = zeros(6,1);
        max_tspcAcc = zeros(6,1);
        
        p_ei = ip(1:3);
        r_ei = ip(4:6);
        p_ef = fp(1:3);
        r_ef = fp(4:6);
        
        Dp = p_ef-p_ei;
        Dr = r_ef-r_ei;

        D = [Dp;Dr];

        klnv = 1;
        klna = 8;

        komg = 0.8;
        kalf = 2;

        max_tspcVel(1:3) = klnv;
        max_tspcVel(4:6) = komg;

        max_tspcAcc(1:3) = klna;
        max_tspcAcc(4:6) = kalf;

        v_prcnt = prcnt(1);
        a_prcnt = prcnt(2);

        % new max vel. & acc.
        kv = v_prcnt*max_tspcVel;
        ka = a_prcnt*max_tspcAcc;
        
        switch trj_profile
            case 1
                tf = max(abs(D)./kv);
    %             vMax = abs(D)/tf;
    %             aMax = 0;
            case 2
                tf = max(max(3*abs(D)./(2*kv)),max(sqrt(6*abs(D)./ka)));
    %             vMax = 3*abs(D)/(2*tf);
    %             aMax = 6*abs(D)/tf^2;
            case 3
                tf = max(max(15*abs(D)./(8*kv)),max(sqrt(10*abs(D)./(sqrt(3)*ka))));
    %             vMax = 15*abs(D)/(8*tf);
    %             aMax = 10*abs(D)/(sqrt(3)*tf^2);
            case 4
                tf = max(max(2*abs(D)./kv),max(sqrt(4*abs(D)./ka)));
    %             vMax = 2*abs(D)/tf;
    %             aMax = 4*abs(D)/tf^2;
            otherwise
                tf = 0;
        end
        
    end

end