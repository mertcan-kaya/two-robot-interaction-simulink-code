function J_e = jacobian(a_i,d_i,d_e,q_pos)

    a2 = a_i(2);
    a3 = a_i(3);
    d2 = d_i(2);
    d4 = d_i(4);
    d6 = d_i(6) + d_e;

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

    C2p3 = cos(q_pos(2)+q_pos(3));
    S2p3 = sin(q_pos(2)+q_pos(3));

    J_e = zeros(6,6);

    J_e(1,1) = -S1 * (a2 + a3 * S2 + (d4 + d6 * C5) * S2p3 + d6 * C2p3*C4*S5) - C1 * (d2 + d6 * S4*S5);
    J_e(2,1) = C1 * (a2 + a3 * S2 + (d4 + d6 * C5) * S2p3 + d6 * C2p3*C4*S5) - S1 * (d2 + d6 * S4*S5);
    J_e(3,1) = 0.0;
    J_e(4,1) = 0.0;
    J_e(5,1) = 0.0;
    J_e(6,1) = 1.0;

    J_e(1,2) = C1 * (a3 * C2 + C2p3 * (d4 + d6 * C5) - d6 * C4*S2p3*S5);
    J_e(2,2) = S1 * (a3 * C2 + C2p3 * (d4 + d6 * C5) - d6 * C4*S2p3*S5);
    J_e(3,2) = -C2 * ((d4 + d6 * C5) * S3 + d6 * C3*C4*S5) -	S2 * (a3 + C3 * (d4 + d6 * C5) - d6 * C4*S3*S5);
    J_e(4,2) = -S1;
    J_e(5,2) = C1;
    J_e(6,2) = 0.0;

    J_e(1,3) = C1 * (C2p3 * (d4 + d6 * C5) -	d6 * C4*S2p3*S5);
    J_e(2,3) = S1 * (C2p3 * (d4 + d6 * C5) -	d6 * C4*S2p3*S5);
    J_e(3,3) = -(d4 + d6 * C5) * S2p3 - d6 * C2p3*C4*S5;
    J_e(4,3) = -S1;
    J_e(5,3) = C1;
    J_e(6,3) = 0.0;

    J_e(1,4) = -d6 * (C4*S1 + C1*C2p3*S4) * S5;
    J_e(2,4) = d6 * (C1*C4 - C2p3*S1*S4) * S5;
    J_e(3,4) = d6 * S2p3*S4*S5;
    J_e(4,4) = C1*S2p3;
    J_e(5,4) = S1*S2p3;
    J_e(1,4) = C2p3;

    J_e(1,5) = -d6 * (C5*S1*S4 +	C1 * (-C2p3*C4*C5 + S2p3*S5));
    J_e(2,5) = d6 * (-C4*C5*S1*S2*S3 + C1*C5*S4 - C3*S1*S2*S5 + C2*S1 * (C3*C4*C5 - S3*S5));
    J_e(3,5) = -d6 * (C4*C5*S2p3 + C2p3*S5);
    J_e(4,5) = -C4*S1 - C1*C2p3*S4;
    J_e(5,5) = C1*C4 - C2p3*S1*S4;
    J_e(6,5) = S2p3*S4;

    J_e(1,6) = 0.0;
    J_e(2,6) = 0.0;
    J_e(3,6) = 0.0;
    J_e(4,6) = -S1*S4*S5 + C1 * (C5*S2p3 + C2p3*C4*S5);
    J_e(5,6) = C5*S1*S2p3 + (C2p3*C4*S1 + C1*S4) * S5;
    J_e(6,6) = C2p3*C5 - C4*S2p3*S5;

end