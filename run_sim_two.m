clc, close all

run_simulink = 1;
open_simulink = 0;
high_quality = 1;

%% Robot 1

trj_space   = 1; % 0:joint, 1:task (operational)
ea_set      = 4; % 1:zxz, 2:zyz, 3:zyx, 4:xyz
trj_type    = 2; % 1: sinusoidal, 2: p2p
trj_profile = 3; % 1:linear, 2:cubic, 3:quintic, 4: trapez, 5: S-trapez
ctrl_space  = 1; % 0:joint, 1:task (operational)
ctc_switch  = 0;
force_ffwd  = 1;
ctrl_at_end = 1;
orn_at_norm = 1;

robot_model_1	= 2;
ee_att_1        = 3;

tfin = '25';
tstp = '0.0001';

cnc_comp = 1; % Coriolis and centrifugal torque compensation
grv_comp = 1; % Gravitational torque compensation
spr_comp = 1; % Spring effect compensation
frc_comp = 0; % Joint friction compensation
prdc_frc = 0; % Predictive friction

animation_on = 1;
robot_1_on = 1;
robot_2_on = 1;
base_on = 1;

% prcnt = [0.3, 0.1];
prcnt = [0.07, 0.05];
if trj_space == 0 % joint space trajectory
    qi = deg2rad([-30;60;12;-20;60;-35]);
    qf = deg2rad([45;-35;115;40;35;15]);
    
    ip = qi;
    fp = qf;
else % task space trajectory
    if ctrl_at_end == 1
        p_ei = [-0.40;1.05;1.0];
    else
        p_ei = [-0.20;1.05;1.0];
%         p_ei = [-0.40;1.05;1.0];
    end
    r_ei = deg2rad([-90;00;00]);
    if trj_type ~= 1
        if ctrl_at_end == 1
            p_ef = [-0.40;1.45;1.0];
        else
            p_ef = [-0.20;1.05;1.0];
%             p_ef = [-0.40;1.45;1.0];
        end
        r_ef = deg2rad([-90;00;00]);
    else
        Dp = [0;-0.2;0];
        Dr = deg2rad([0;0;0]);
        
        p_ef = p_ei+Dp;
        r_ef = r_ei+Dr;
    end
    
    ip = [p_ei;r_ei];
    fp = [p_ef;r_ef];
    
    qi = getInitialPoints(p_ei,r_ei,robot_model_1,ee_att_1,ea_set,1);
end

if ctrl_space == 0      % joint space control
    if ctc_switch == 0  % PID control
        Kp = [3800;10000;7500;3000;1100;500];
        Ki = [0;0;0;0;0;0];
        Kd = [10;10;10;10;10;10];
    else                % Computed torque control
        Kp = [2250;2500;900;300;250;250];
        Ki = [30;110;90;30;15;15];
        Kd = [85;100;30;10;10;10];
    end
else                    % task space control
    if ctc_switch == 0  % PID control
        Kp = [2500;2500;2500;550;550;550];
        Kd = [5;15;15;2;2;2];
        Ki = [10;10;10;3;3;3];
    else                % Computed torque control
        Kp = [2550;2250;2750;1150;850;650];
        Ki = [20;20;20;5;5;5];
        Kd = [10;10;10;7;7;7];
    end
end
Km = [1;1;1;1;1;1];

switch_val = 30;
% switch_val = 00;
if ctrl_at_end == 1
    if orn_at_norm == 1
        Sx_diag = [1;1;0;1;1;1];
        Sf_diag = [0;0;1;0;0;0];
    else
        Sx_diag = [1;1;0;1;1;1];
        Sf_diag = [0;0;1;0;0;0];
    end
else
    Sx_diag = [1;0;1;1;1;1];
    Sf_diag = [0;1;0;0;0;0];
end

% Solid object stiffness
% K = 15000; % Stifness
K = 150000; % Stifness
K = 15000; % Stifness
% K = 0; % Stifness
Kc = 0.001; % Compliance (inverse of stiffness)
% Kc = 0.05; % Compliance (inverse of stiffness)
% Kc = 0.1; % Compliance (inverse of stiffness)
Kc = 0.005; % Compliance (inverse of stiffness)

if ctrl_at_end == 1
    f_des = [0;0;60;0;0;0];
    
    Kpf = [0;10;3;0;0;0];
    Kif = [0;0;1;0;0;0];
    Kdf = [0;0;10;0;0;0];
else
    f_des = [0;60;0;0;0;0];
    
    Kpf = [0;3;0;0;0;0];
    Kif = [0;0.05;0;0;0;0];
    Kdf = [0;10;0;0;0;0];
end
        
q_posIni = qi;
rad2deg(qi)

if trj_type ~= 1
    rpt_num = 1;
    tf = computeTrajectoryTime(ip,fp,prcnt,trj_space,trj_profile,robot_model_1)
else
    rpt_num = 2;
    period = 15;

    tf = rpt_num * period
end

%% Robot 2

robot_model_2 = 3;
ee_att_2    = 4;

trj_space2   = 1; % 0:joint, 1:task (operational)
ea_set2      = 4; % 1:zxz, 2:zyz, 3:zyx, 4:xyz
trj_type2    = 1; % 1: sinusoidal, 2: p2p
trj_profile2 = 3; % 1:linear, 2:cubic, 3:quintic, 4: trapez, 5: S-trapez

if trj_space2 == 0 % joint space trajectory
    qi2 = deg2rad([-30;60;12;-20;60;-35]);
    qf2 = deg2rad([45;-35;115;40;35;15]);
    
    ip2 = qi2;
    fp2 = qf2;
else % task space trajectory
    p_ei2 = [-0.05;-1.10;1.0];
    r_ei2 = deg2rad([60;00;00]);
%     r_ei2 = deg2rad([90;00;00]);
    if trj_type2 ~= 1
        p_ef2 = [0.85;-0.75;1.2];
        r_ef2 = deg2rad([110;-20;20]);
    else
        Dp = [0;0;0];
        Dr = deg2rad([20;00;00]);
        
        p_ef2 = p_ei2+Dp;
        r_ef2 = r_ei2+Dr;
    end
    
    ip2 = [p_ei2;r_ei2];
    fp2 = [p_ef2;r_ef2];
    
    qi2 = getInitialPoints(p_ei2,r_ei2,robot_model_2,ee_att_2,ea_set,1)
end

q_posIni2 = qi2;
rad2deg(qi2)

if trj_type2 ~= 1
    rpt_num = 1;
    tf2 = computeTrajectoryTime(ip2,fp2,prcnt,trj_space2,trj_profile2,robot_model_2)
else
    rpt_num2 = 10;
    period2 = 3;

    tf2 = rpt_num2 * period2
end

no_trj = 1;
no_trj2 = 1;
% tf = 0.0001;
% tf2 = 0.0001;

%% Simulink commands

if open_simulink == 1
    open_system('sim_rx160_two')
end

if run_simulink == 1
    model = 'sim_rx160_two';
    load_system(model)
    out = sim(model,'StartTime','0','StopTime',tfin,'FixedStep',tstp); % sim('modelName','StartTime','0','StopTime','10','FixedStep','0.2');
end

if animation_on == 1 && exist('out','var') == 1
%     animate_sim(robot_model_1,ee_att_1,robot_model_2,ee_att_2,high_quality,robot_1_on,robot_2_on,base_on,out)
%     animate_sim_par(robot_model_1,ee_att_1,robot_model_2,ee_att_2,high_quality,robot_1_on,robot_2_on,base_on,out)
%     animate_sim_parsmpl(out)

    addpath('D:\Cloud Storage\MATLAB Drive\cad_files\rx160_series\visual');
    
    % Import data from Simulink
    q1 = out.jointPosFbk.Data(:,1)';
    q2 = out.jointPosFbk.Data(:,2)';
    q3 = out.jointPosFbk.Data(:,3)';
    q4 = out.jointPosFbk.Data(:,4)';
    q5 = out.jointPosFbk.Data(:,5)';
    q6 = out.jointPosFbk.Data(:,6)';
    time = 1:length(q1);

    % Robot parameters
    n = 6;

    % Load CAD files
    temp = load('base_link.mat');   V0 = temp.modelstruct.V;   F0 = temp.modelstruct.F;   C0 = temp.modelstruct.C;
    temp = load('link_1.mat');      V1 = temp.modelstruct.V;   F1 = temp.modelstruct.F;   C1 = temp.modelstruct.C;
    temp = load('link_2.mat');      V2 = temp.modelstruct.V;   F2 = temp.modelstruct.F;   C2 = temp.modelstruct.C;
    temp = load('link_3.mat');      V3 = temp.modelstruct.V;   F3 = temp.modelstruct.F;   C3 = temp.modelstruct.C;
    temp = load('link_4.mat');      V4 = temp.modelstruct.V;   F4 = temp.modelstruct.F;   C4 = temp.modelstruct.C;
    temp = load('link_5.mat');      V5 = temp.modelstruct.V;   F5 = temp.modelstruct.F;   C5 = temp.modelstruct.C;
    temp = load('link_6.mat');      V6 = temp.modelstruct.V;   F6 = temp.modelstruct.F;   C6 = temp.modelstruct.C;

    clear temp

    % Generate animation figure
    animFig = figure('Name','Animation Plot','WindowState', 'maximized');
    animPlot = axes('Parent',animFig);
    light                               % add a default light
    view(3)                             % Isometric view

    xlabel(animPlot,'x')
    ylabel(animPlot,'y')
    zlabel(animPlot,'z')
    grid(animPlot,'on')

    % CAD properties
    p0 = patch(animPlot,'Faces', F0, 'Vertices', V0);
    set(p0, 'facec', 'flat');                  % Set the face color flat
    set(p0, 'FaceColor', C0);           % Set the color (from file)
    set(p0, 'EdgeColor', 'none');              % Set the edge color
    p1 = patch(animPlot,'Faces', F1, 'Vertices', V1);
    set(p1, 'facec', 'flat');                  % Set the face color flat
    set(p1, 'FaceColor', C1);           % Set the color (from file)
    set(p1, 'EdgeColor', 'none');              % Set the edge color
    p2 = patch(animPlot,'Faces', F2, 'Vertices', V2);
    set(p2, 'facec', 'flat');                  % Set the face color flat
    set(p2, 'FaceColor', C2);           % Set the color (from file)
    set(p2, 'EdgeColor', 'none');              % Set the edge color
    p3 = patch(animPlot,'Faces', F3, 'Vertices', V3);
    set(p3, 'facec', 'flat');                  % Set the face color flat
    set(p3, 'FaceColor', C3);           % Set the color (from file)
    set(p3, 'EdgeColor', 'none');              % Set the edge color
    p4 = patch(animPlot,'Faces', F4, 'Vertices', V4);
    set(p4, 'facec', 'flat');                  % Set the face color flat
    set(p4, 'FaceColor', C4);           % Set the color (from file)
    set(p4, 'EdgeColor', 'none');              % Set the edge color
    p5 = patch(animPlot,'Faces', F5, 'Vertices', V5);
    set(p5, 'facec', 'flat');                  % Set the face color flat
    set(p5, 'FaceColor', C5);           % Set the color (from file)
    set(p5, 'EdgeColor', 'none');              % Set the edge color
    p6 = patch(animPlot,'Faces', F6, 'Vertices', V6);
    set(p6, 'facec', 'flat');                  % Set the face color flat
    set(p6, 'FaceColor', C6);           % Set the color (from file)
    set(p6, 'EdgeColor', 'none');              % Set the edge color
    
    v0x = V0(:,1);
    v0y = V0(:,2);
    v0z = V0(:,3);
    
    v1x = V1(:,1);
    v1y = V1(:,2);
    v1z = V1(:,3);
    
    v2x = V2(:,1);
    v2y = V2(:,2);
    v2z = V2(:,3);
    
    v3x = V3(:,1);
    v3y = V3(:,2);
    v3z = V3(:,3);
    
    v4x = V4(:,1);
    v4y = V4(:,2);
    v4z = V4(:,3);
    
    v5x = V5(:,1);
    v5y = V5(:,2);
    v5z = V5(:,3);
    
    v6x = V6(:,1);
    v6y = V6(:,2);
    v6z = V6(:,3);
    
    piOver2 = pi/2;

    a1 = 0;     a2 = 0.150; a3 = 0.825;
    d1 = 0.550; d2 = 0;     d4 = 0.625; d6 = 0.110;

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

    parfor t = 1:length(time)
%     parfor t = 1:1
%     for t = 1:1
        
        hold(animPlot,'on')
        
        TI_0	= eye(4);

        TI_1 = TI_0 ...
            * [1 0 0 0;0 cos(A1) -sin(A1) 0;0 sin(A1) cos(A1) 0;0 0 0 1] ...
            * [1 0 0 a1;0 1 0 0;0 0 1 0;0 0 0 1] ...
            * [cos(T1+q1(t)) -sin(T1+q1(t))	0 0;sin(T1+q1(t)) cos(T1+q1(t)) 0 0;0 0 1 0;0 0 0 1] ...
            * [1 0 0 0;0 1 0 0;0 0 1 d1;0 0 0 1];

        TI_2 = TI_1 ...
            * [1 0 0 0;0 cos(A2) -sin(A2) 0;0 sin(A2) cos(A2) 0;0 0 0 1] ...
            * [1 0 0 a2;0 1 0 0;0 0 1 0;0 0 0 1] ...
            * [cos(T2+q2(t)) -sin(T2+q2(t))	0 0;sin(T2+q2(t)) cos(T2+q2(t)) 0 0;0 0 1 0;0 0 0 1] ...
            * [1 0 0 0;0 1 0 0;0 0 1 d2;0 0 0 1];

        TI_3 = TI_2 ...
            * [1 0 0 0;0 cos(A3) -sin(A3) 0;0 sin(A3) cos(A3) 0;0 0 0 1] ...
            * [1 0 0 a3;0 1 0 0;0 0 1 0;0 0 0 1] ...
            * [cos(T3+q3(t)) -sin(T3+q3(t))	0 0;sin(T3+q3(t)) cos(T3+q3(t)) 0 0;0 0 1 0;0 0 0 1] ...
            * [1 0 0 0;0 1 0 0;0 0 1 d3;0 0 0 1];

        TI_4 = TI_3 ...
            * [1 0 0 0;0 cos(A4) -sin(A4) 0;0 sin(A4) cos(A4) 0;0 0 0 1] ...
            * [1 0 0 a4;0 1 0 0;0 0 1 0;0 0 0 1] ...
            * [cos(T4+q4(t)) -sin(T4+q4(t))	0 0;sin(T4+q4(t)) cos(T4+q4(t)) 0 0;0 0 1 0;0 0 0 1] ...
            * [1 0 0 0;0 1 0 0;0 0 1 d4;0 0 0 1];

        TI_5 = TI_4 ...
            * [1 0 0 0;0 cos(A5) -sin(A5) 0;0 sin(A5) cos(A5) 0;0 0 0 1] ...
            * [1 0 0 a5;0 1 0 0;0 0 1 0;0 0 0 1] ...
            * [cos(T5+q5(t)) -sin(T5+q5(t))	0 0;sin(T5+q5(t)) cos(T5+q5(t)) 0 0;0 0 1 0;0 0 0 1] ...
            * [1 0 0 0;0 1 0 0;0 0 1 d5;0 0 0 1];

        TI_6 = TI_5 ...
            * [1 0 0 0;0 cos(A6) -sin(A6) 0;0 sin(A6) cos(A6) 0;0 0 0 1] ...
            * [1 0 0 a6;0 1 0 0;0 0 1 0;0 0 0 1] ...
            * [cos(T6+q6(t)) -sin(T6+q6(t))	0 0;sin(T6+q6(t)) cos(T6+q6(t)) 0 0;0 0 1 0;0 0 0 1] ...
            * [1 0 0 0;0 1 0 0;0 0 1 d6;0 0 0 1];
        
% TFbkI_h_1
% V0
% [V0,ones(size(V0(:,1)))]*TFbkI_h_1(:,:,1)'
        nv0 = [V0,ones(size(V0(:,1)))]*TI_0';
        nv1 = [V1,ones(size(V1(:,1)))]*TI_1';
        nv2 = [V2,ones(size(V2(:,1)))]*TI_2';
        nv3 = [V3,ones(size(V3(:,1)))]*TI_3';
        nv4 = [V4,ones(size(V4(:,1)))]*TI_4';
        nv5 = [V5,ones(size(V5(:,1)))]*TI_5';
        nv6 = [V6,ones(size(V6(:,1)))]*TI_6';

    end
%         nv0x = TFbkI_h_1(1,1,1)*v0x+TFbkI_h_1(1,2,1)*v0y+TFbkI_h_1(1,3,1)*v0z+TFbkI_h_1(1,4,1);
%         nv0y = TFbkI_h_1(2,1,1)*v0x+TFbkI_h_1(2,2,1)*v0y+TFbkI_h_1(2,3,1)*v0z+TFbkI_h_1(2,4,1);
%         nv0z = TFbkI_h_1(3,1,1)*v0x+TFbkI_h_1(3,2,1)*v0y+TFbkI_h_1(3,3,1)*v0z+TFbkI_h_1(3,4,1);
%         
%         nv1x = TFbkI_h_1(1,1,2)*v1x+TFbkI_h_1(1,2,2)*v1y+TFbkI_h_1(1,3,2)*v1z+TFbkI_h_1(1,4,2);
%         nv1y = TFbkI_h_1(2,1,2)*v1x+TFbkI_h_1(2,2,2)*v1y+TFbkI_h_1(2,3,2)*v1z+TFbkI_h_1(2,4,2);
%         nv1z = TFbkI_h_1(3,1,2)*v1x+TFbkI_h_1(3,2,2)*v1y+TFbkI_h_1(3,3,2)*v1z+TFbkI_h_1(3,4,2);
%         
%         nv2x = TFbkI_h_1(1,1,3)*v2x+TFbkI_h_1(1,2,3)*v2y+TFbkI_h_1(1,3,3)*v2z+TFbkI_h_1(1,4,3);
%         nv2y = TFbkI_h_1(2,1,3)*v2x+TFbkI_h_1(2,2,3)*v2y+TFbkI_h_1(2,3,3)*v2z+TFbkI_h_1(2,4,3);
%         nv2z = TFbkI_h_1(3,1,3)*v2x+TFbkI_h_1(3,2,3)*v2y+TFbkI_h_1(3,3,3)*v2z+TFbkI_h_1(3,4,3);
%         
%         nv3x = TFbkI_h_1(1,1,4)*v3x+TFbkI_h_1(1,2,4)*v3y+TFbkI_h_1(1,3,4)*v3z+TFbkI_h_1(1,4,4);
%         nv3y = TFbkI_h_1(2,1,4)*v3x+TFbkI_h_1(2,2,4)*v3y+TFbkI_h_1(2,3,4)*v3z+TFbkI_h_1(2,4,4);
%         nv3z = TFbkI_h_1(3,1,4)*v3x+TFbkI_h_1(3,2,4)*v3y+TFbkI_h_1(3,3,4)*v3z+TFbkI_h_1(3,4,4);
%         
%         nv4x = TFbkI_h_1(1,1,5)*v4x+TFbkI_h_1(1,2,5)*v4y+TFbkI_h_1(1,3,5)*v4z+TFbkI_h_1(1,4,5);
%         nv4y = TFbkI_h_1(2,1,5)*v4x+TFbkI_h_1(2,2,5)*v4y+TFbkI_h_1(2,3,5)*v4z+TFbkI_h_1(2,4,5);
%         nv4z = TFbkI_h_1(3,1,5)*v4x+TFbkI_h_1(3,2,5)*v4y+TFbkI_h_1(3,3,5)*v4z+TFbkI_h_1(3,4,5);
%         
%         nv5x = TFbkI_h_1(1,1,6)*v5x+TFbkI_h_1(1,2,6)*v5y+TFbkI_h_1(1,3,6)*v5z+TFbkI_h_1(1,4,6);
%         nv5y = TFbkI_h_1(2,1,6)*v5x+TFbkI_h_1(2,2,6)*v5y+TFbkI_h_1(2,3,6)*v5z+TFbkI_h_1(2,4,6);
%         nv5z = TFbkI_h_1(3,1,6)*v5x+TFbkI_h_1(3,2,6)*v5y+TFbkI_h_1(3,3,6)*v5z+TFbkI_h_1(3,4,6);
%         
%         nv6x = TFbkI_h_1(1,1,7)*v6x+TFbkI_h_1(1,2,7)*v6y+TFbkI_h_1(1,3,7)*v6z+TFbkI_h_1(1,4,7);
%         nv6y = TFbkI_h_1(2,1,7)*v6x+TFbkI_h_1(2,2,7)*v6y+TFbkI_h_1(2,3,7)*v6z+TFbkI_h_1(2,4,7);
%         nv6z = TFbkI_h_1(3,1,7)*v6x+TFbkI_h_1(3,2,7)*v6y+TFbkI_h_1(3,3,7)*v6z+TFbkI_h_1(3,4,7);
       nnv0 = nv0(:,1:3);
       nnv1 = nv1(:,1:3);
       nnv2 = nv2(:,1:3);
       nnv3 = nv3(:,1:3);
       nnv4 = nv4(:,1:3);
       nnv5 = nv5(:,1:3);
       nnv6 = nv6(:,1:3);
       
    for t = 1:length(time)
        
%         set(p0,'Vertices',nv0(:,1:3))
%         set(p1,'Vertices',nv1(:,1:3))
%         set(p2,'Vertices',nv2(:,1:3))
%         set(p3,'Vertices',nv3(:,1:3))
%         set(p4,'Vertices',nv4(:,1:3))
%         set(p5,'Vertices',nv5(:,1:3))
%         set(p6,'Vertices',nv6(:,1:3))
            
        set(p0,'Vertices',nnv0)
        set(p1,'Vertices',nnv1)
        set(p2,'Vertices',nnv2)
        set(p3,'Vertices',nnv3)
        set(p4,'Vertices',nnv4)
        set(p5,'Vertices',nnv5)
        set(p6,'Vertices',nnv6)
%         set(p0,'Vertices',[nv0x nv0y nv0z])
%         set(p1,'Vertices',[nv1x nv1y nv1z])
%         set(p2,'Vertices',[nv2x nv2y nv2z])
%         set(p3,'Vertices',[nv3x nv3y nv3z])
%         set(p4,'Vertices',[nv4x nv4y nv4z])
%         set(p5,'Vertices',[nv5x nv5y nv5z])
%         set(p6,'Vertices',[nv6x nv6y nv6z])

%         set(p0,'Vertices',V0)
%         set(p1,'Vertices',V1)
%         set(p2,'Vertices',V2)
%         set(p3,'Vertices',V3)
%         set(p4,'Vertices',V4)
%         set(p5,'Vertices',V5)
%         set(p6,'Vertices',V6)
        
        hold(animPlot,'off')

        set(animPlot,	'Projection','perspective', ...
                        'PlotBoxAspectRatio',[1 1 1], ...
                        'DataAspectRatio',[1 1 1]);
        drawnow;
    end

end