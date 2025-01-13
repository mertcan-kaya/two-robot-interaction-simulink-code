function q_i = invGeo(T0_e,a_i,alpha_i,d_i,d_e,thetaPlus_i,q_ref,config)

    Px = T0_e(1,4) - T0_e(1,3) * (d_i(6) + d_e);
    Py = T0_e(2,4) - T0_e(2,3) * (d_i(6) + d_e);
    Pz = T0_e(3,4) - T0_e(3,3) * (d_i(6) + d_e) - d_i(1);

    % theta 1
%         if robot_model ~= 1
        q1 = atan2(Py,Px);
%         else
%             % implement type 2
%         end

    % theta 2 & 3

    W = -d_i(4);
    X = -cos(q1)*Px - sin(q1)*Py + a_i(2);
    Y = Pz;
    Z1 = -a_i(3);
    Z2 = 0;

    B1 = 2*(Z1*Y+Z2*X);
    B2 = 2*(Z1*X-Z2*Y);
    B3 = W^2-X^2-Y^2-Z1^2-Z2^2;

    e = [-1,1];

    q2i = zeros(1,2);
    for i = 1:2
        C2 = (B2*B3-e(i)*B1*sqrt(B1^2+B2^2-B3^2))/(B1^2+B2^2);
        S2 = (B1*B3+e(i)*B2*sqrt(B1^2+B2^2-B3^2))/(B1^2+B2^2);

        q2i(i) = atan2(S2,C2);
    end

%     q2a = q2i(1);
    q2b = q2i(2);

    q2 = q2b;

    S3 = (X*C2+Y*S2+Z1)/W;
    C3 = (X*S2-Y*C2+Z2)/W;

    q3 = atan2(S3,C3);

    %% b) Computation of theta4, theta5, theta6

    q1to3 = [q1;q2;q3;0;0;0];

    Ti_h = zeros(4,4,3);
    for i = 1:3
        CA_h = cos(alpha_i(i));
        SA_h = sin(alpha_i(i));
        CT_h = cos(thetaPlus_i(i) + q1to3(i));
        ST_h = sin(thetaPlus_i(i) + q1to3(i));

        Ti_h(:,:,i) = [	CT_h	, CA_h * ST_h	, SA_h * ST_h	, -a_i(i) * CT_h
                        -ST_h	, CA_h * CT_h	, SA_h * CT_h	, a_i(i) * ST_h
                        0		, -SA_h			, CA_h			, -d_i(i)
                        0		, 0				, 0				, 1				];

    end

    T3_0 = Ti_h(:,:,3)*Ti_h(:,:,2)*Ti_h(:,:,1);

    R3_0 = T3_0(1:3,1:3);

    sna = T0_e(1:3,1:3);

    FGH = R3_0*sna;

    F = FGH(:,1);
    G = FGH(:,2);
    H = FGH(:,3);

    % theta 4
    q4a = atan2(H(3),H(1));

    if config == 1
        if (abs(q_ref(4) - (q4a - 2*pi)) < abs(q_ref(4) - q4a))
            q4 = q4a-2*pi;
        else
            q4 = q4a+pi;
        end
    else
        if (abs(q_ref(4) - (q4a - pi)) < abs(q_ref(4) - q4a))
            q4 = q4a - pi;
        elseif (abs(q_ref(4) - (q4a + pi / 2)) < abs(q_ref(4) - q4a))
            q4 = q4a + pi / 2;
        elseif (abs(q_ref(4) - (q4a - pi / 2)) < abs(q_ref(4) - q4a))
            q4 = q4a - pi / 2;
        elseif (abs(q_ref(4) - (q4a + 2 * pi)) < abs(q_ref(4) - q4a))
            q4 = q4a + 2 * pi;
        else
            q4 = q4a;
        end
    end
    
    % theta 5
    S5 = sin(q4)*H(3)+cos(q4)*H(1);
    C5 = -H(2);

    q5 = atan2(S5,C5);

    % theta 6
    S6 = cos(q4)*F(3)-sin(q4)*F(1);
    C6 = cos(q4)*G(3)-sin(q4)*G(1);

    q6a = atan2(S6,C6);
    if (abs(q_ref(6) - (q6a - 2*pi)) < abs(q_ref(6) - q6a))
        q6 = q6a - 2*pi;
    else
        q6 = atan2(S6,C6);
    end

    % Result
    q_i = [q1;q2;q3;q4;q5;q6]+thetaPlus_i;

end