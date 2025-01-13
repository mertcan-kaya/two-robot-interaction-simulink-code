function [a_i,alpha_i,d_i,thetaPlus_i] = kinematicParametersOrigin(robot_model)

    piOver2 = pi/2;

    switch robot_model
        case 1 % TX90
            a1 = 0;     a2 = 0.050; a3 = 0.425;
            d1 = 0.478; d2 = 0.050; d4 = 0.425; d6 = 0.100;
        case 2 % RX160
            a1 = 0;     a2 = 0.150; a3 = 0.825;
            d1 = 0.550; d2 = 0;     d4 = 0.625; d6 = 0.110;
        case 3 % RX160L
            a1 = 0;     a2 = 0.150; a3 = 0.825;
            d1 = 0.550; d2 = 0;     d4 = 0.925; d6 = 0.110;
        otherwise
            a1 = 0; a2 = 0; a3 = 0;
            d1 = 0; d2 = 0; d4 = 0; d6 = 0;
    end

    A2 = -piOver2; A4 = piOver2; A5 = -piOver2; A6 = piOver2;
    T2 = -piOver2; T3 = piOver2;

    a4 = 0; a5 = 0; a6 = 0;
    d3 = 0; d5 = 0;
    A1 = 0; A3 = 0; 
    T1 = 0; T4 = 0; T5 = 0; T6 = 0;
    
    % General DH
    a_i     = [a1;a2;a3;a4;a5;a6];
    alpha_i = [A1;A2;A3;A4;A5;A6];
    d_i     = [d1;d2;d3;d4;d5;d6];
    thetaPlus_i = [T1;T2;T3;T4;T5;T6];

end