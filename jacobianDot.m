function J_eDot = jacobianDot(a_i,d_i,d_e,q_pos,q_vel)

    a2 = a_i(2);
    a3 = a_i(3);
    d2 = d_i(2);
    d4 = d_i(4);
    d6 = d_i(6) + d_e;

    qd1 = q_vel(1);
    qd2 = q_vel(2);
    qd3 = q_vel(3);
    qd4 = q_vel(4);
    qd5 = q_vel(5);

    C1 = cos(q_pos(1));
    C2 = cos(q_pos(2));
    C3 = cos(q_pos(3));
    C4 = cos(q_pos(4));
    C5 = cos(q_pos(5));

    S1 = sin(q_pos(1));
    S2 = sin(q_pos(2));
    S3 = sin(q_pos(3));
    S4 = sin(q_pos(4));
    S5 = sin(q_pos(5));

    C2p3	= cos(q_pos(2)+q_pos(3));
    S2p3	= sin(q_pos(2)+q_pos(3));

    J_eDot = zeros(6,6);

    J_eDot(1,1) = -(a2 * qd1 * C1) + d2 * qd1 * S1 - a3 * qd2 * C2*S1 - d4 * (qd2 + qd3) * C2p3 * S1 - a3 * qd1 * C1*S2 ...
                - d4 * qd1 * C1*S2p3 - d6 * (C2*C5 * (C3 * (qd2 + qd3 + qd5 * C4) * S1 + qd1 * C1*S3) ...
                + C1 * (qd1 * C3*C5*S2 + qd5 * C5*S4 + (qd4 + qd1 * C2p3) * C4*S5) - S1 * ((qd2 + qd3 + qd5 * C4) * C5*S2*S3 ...
                + ((qd5 + (qd2 + qd3) * C4) * S2p3 + (qd1 + qd4 * C2p3) * S4) * S5));
    J_eDot(2,1) = -(S1 * (a2 * qd1 + a3 * qd1 * S2 + qd1 * (d4 + d6 * C5) * S2p3 + d6 * qd5 * C5*S4 ...
                + d6 * (qd4 + qd1 * C2p3) * C4*S5)) + C1 * (-(d2 * qd1) + a3 * qd2 * C2 ...
                + C2p3 * (d4 * (qd2 + qd3) + d6 * (qd2 + qd3 + qd5 * C4) * C5) ...
                - d6 * ((qd5 + (qd2 + qd3) * C4) * S2p3 + (qd1 + qd4 * C2p3) * S4) * S5);
    J_eDot(3,1) = 0.0;
    J_eDot(4,1) = 0.0;
    J_eDot(5,1) = 0.0;
    J_eDot(6,1) = 0.0;

    J_eDot(1,2) = -(qd1 * S1 * (a3 * C2 + C2p3 * (d4 + d6 * C5) - d6 * C4*S2p3*S5)) ...
                - C1 * (a3 * qd2 * S2 + d6 * C2p3 * (qd5 + (qd2 + qd3) * C4) * S5 + S2p3 * (d4 * (qd2 + qd3) ...
                + d6 * (qd2 + qd3 + qd5 * C4) * C5 - d6 * qd4 * S4*S5));
    J_eDot(2,2) = qd1 * C1 * (a3 * C2 + C2p3 * (d4 + d6 * C5) - d6 * C4*S2p3*S5) ...
                - S1 * (a3 * qd2 * S2 + d6 * C2p3 * (qd5 + (qd2 + qd3) * C4) * S5 ...
                + S2p3 * (d4 * (qd2 + qd3) + d6 * (qd2 + qd3 + qd5 * C4) * C5 - d6 * qd4 * S4*S5));
    J_eDot(3,2) = -(C2 * (a3 * qd2 + C3 * (d4 * (qd2 + qd3) + d6 * (qd2 + qd3 + qd5 * C4) * C5) ...
                - d6 * ((qd5 + (qd2 + qd3) * C4) * S3 + qd4 * C3*S4) * S5)) + S2 * (d6 * C3 * (qd5 + (qd2 + qd3) * C4) * S5 ...
                + S3 * (d4 * (qd2 + qd3) + d6 * (qd2 + qd3 + qd5 * C4) * C5 - d6 * qd4 * S4*S5));
    J_eDot(4,2) = -(qd1 * C1);
    J_eDot(5,2) = -(qd1 * S1);
    J_eDot(6,2) = 0.0;

    J_eDot(1,3) = C2p3 * (-(qd1 * (d4 + d6 * C5) * S1) - d6 * C1 * (qd5 + (qd2 + qd3) * C4) * S5) ...
                + S2p3 * (d6 * qd1 * C4*S1*S5 + C1 * (-(d4 * (qd2 + qd3)) - d6 * (qd2 + qd3 + qd5 * C4) * C5 + d6 * qd4 * S4*S5));
    J_eDot(2,3) = qd1 * C1 * (C2p3 * (d4 + d6 * C5) - d6 * C4*S2p3*S5) - S1 * (d6 * C2p3 * (qd5 + (qd2 + qd3) * C4) * S5 ...
                + S2p3 * (d4 * (qd2 + qd3) + d6 * (qd2 + qd3 + qd5 * C4) * C5 - d6 * qd4 * S4*S5));
    J_eDot(3,3) = S2 * (d6 * C3 * (qd5 + (qd2 + qd3) * C4) * S5 + S3 * (d4 * (qd2 + qd3) + d6 * (qd2 + qd3 + qd5 * C4) * C5 ...
                - d6 * qd4 * S4*S5)) + C2 * (d6 * (qd5 + (qd2 + qd3) * C4) * S3*S5 ...
                + C3 * (-(d4 * (qd2 + qd3)) - d6 * (qd2 + qd3 + qd5 * C4) * C5 + d6 * qd4 * S4*S5));
    J_eDot(4,3) = -(qd1 * C1);
    J_eDot(5,3) = -(qd1 * S1);
    J_eDot(6,3) = 0.0;

    J_eDot(1,4) = d6 * (-(C4 * (qd5 * C5*S1 + C1 * (qd1 + qd4 * C2p3) * S5)) ...
                + S4 * ((qd4 + qd1 * C2p3) * S1*S5 + C1 * (-(qd5 * C2p3*C5) + (qd2 + qd3) * S2p3*S5)));
    J_eDot(2,4) = d6 * (C1 * (qd5 * C4*C5 - (qd4 + qd1 * C2p3) * S4*S5) + S1 * (-(qd5 * C2p3*C5*S4) ...
                + (-((qd1 + qd4 * C2p3) * C4) + (qd2 + qd3) * S2p3*S4) * S5));
    J_eDot(3,4) = d6 * (C3 * (qd5 * C5*S2*S4 + (qd4 * C4*S2 + (qd2 + qd3) * C2*S4) * S5) ...
                + S3 * (-((qd2 + qd3) * S2*S4*S5) + C2 * (qd5 * C5*S4 + qd4 * C4*S5)));
    J_eDot(4,4) = (qd2 + qd3) * C1*C2p3 - qd1 * S1*S2p3;
    J_eDot(5,4) = (qd2 + qd3) * C2p3*S1 + qd1 * C1*S2p3;
    J_eDot(6,4) = -((qd2 + qd3) * S2p3);

    J_eDot(1,5) = -(d6 * (-(S1 * (qd1 * S2p3 + qd5 * S4) * S5) + C1 * (C5 * (qd5 * S2p3 + (qd1 + qd4 * C2p3) * S4) ...
                + (qd2 + qd3) * C2p3*S5) + C4 * (C5 * ((qd4 + qd1 * C2p3) * S1 + (qd2 + qd3) * C1*S2p3) + qd5 * C1*C2p3*S5)));
    J_eDot(2,5) = d6 * (S1 * (C5 * (-((qd5 + (qd2 + qd3) * C4) * S2p3) - (qd1 + qd4 * C2p3) * S4) ...
                - C2p3 * (qd2 + qd3 + qd5 * C4) * S5) + C1 * ((qd4 + qd1 * C2p3) * C4*C5 - (qd1 * S2p3 + qd5 * S4) * S5));
    J_eDot(3,5) = d6 * (S2 * (C5 * ((qd5 + (qd2 + qd3) * C4) * S3 + qd4 * C3*S4) + C3 * (qd2 + qd3 + qd5 * C4) * S5) ...
                + C2 * (-(C3 * (qd5 + (qd2 + qd3) * C4) * C5) + S3 * (qd4 * C5*S4 + (qd2 + qd3 + qd5 * C4) * S5)));
    J_eDot(4,5) = (qd4 + qd1 * C2p3) * S1*S4 + C1 * (-((qd1 + qd4 * C2p3) * C4) + (qd2 + qd3) * S2p3*S4);
    J_eDot(5,5) = -((qd1 + qd4 * C2p3) * C4*S1) + (-(C1 * (qd4 + qd1 * C2p3)) + (qd2 + qd3) * S1*S2p3) * S4;
    J_eDot(6,5) = qd4 * C4*S2p3 + (qd2 + qd3) * C2p3*S4;

    J_eDot(1,6) = 0.0;
    J_eDot(2,6) = 0.0;
    J_eDot(3,6) = 0.0;
    J_eDot(4,6) = -(S1 * (C5 * (qd1 * S2p3 + qd5 * S4) + (qd4 + qd1 * C2p3) * C4*S5)) +	C1 * (C2p3 * (qd2 + qd3 + qd5 * C4) * C5 ...
                - ((qd5 + (qd2 + qd3) * C4) * S2p3 + (qd1 + qd4 * C2p3) * S4) * S5);
    J_eDot(5,6) = C2*C5 * (C3 * (qd2 + qd3 + qd5 * C4) * S1 + qd1 * C1*S3) ...
                + C1 * (qd1 * C3*C5*S2 + qd5 * C5*S4 + (qd4 + qd1 * C2p3) * C4*S5) ...
                - S1 * ((qd2 + qd3 + qd5 * C4) * C5*S2*S3 +	((qd5 + (qd2 + qd3) * C4) * S2p3 + (qd1 + qd4 * C2p3) * S4) * S5);
    J_eDot(6,6) = -((qd2 + qd3 + qd5 * C4) * C5*S2p3) +	(-(C2p3 * (qd5 + (qd2 + qd3) * C4)) + qd4 * S2p3*S4) * S5;

end