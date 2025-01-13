function animate_sim_parsmpl(out)

    addpath('D:\Cloud Storage\MATLAB Drive\cad_files\rx160_series\visual');
    
    % Import data from Simulink
    q_posFbk_1 = out.jointPosFbk.Data';
    time = 1:length(q_posFbk_1);

    % Robot parameters
    n = 6;

    % Load CAD files
    temp = load('base_link.mat');   s0 = temp.modelstruct;
    temp = load('link_1.mat');      s1 = temp.modelstruct;
    temp = load('link_2.mat');      s2 = temp.modelstruct;
    temp = load('link_3.mat');      s3 = temp.modelstruct;
    temp = load('link_4.mat');      s4 = temp.modelstruct;
    temp = load('link_5.mat');      s5 = temp.modelstruct;
    temp = load('link_6.mat');      s6 = temp.modelstruct;

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
    p0 = patch(animPlot,'Faces', s0.F, 'Vertices', s0.V);
    set(p0, 'facec', 'flat');                  % Set the face color flat
    set(p0, 'FaceColor', s0.C);           % Set the color (from file)
    set(p0, 'EdgeColor', 'none');              % Set the edge color
    p1 = patch(animPlot,'Faces', s1.F, 'Vertices', s1.V);
    set(p1, 'facec', 'flat');                  % Set the face color flat
    set(p1, 'FaceColor', s1.C);           % Set the color (from file)
    set(p1, 'EdgeColor', 'none');              % Set the edge color
    p2 = patch(animPlot,'Faces', s2.F, 'Vertices', s2.V);
    set(p2, 'facec', 'flat');                  % Set the face color flat
    set(p2, 'FaceColor', s2.C);           % Set the color (from file)
    set(p2, 'EdgeColor', 'none');              % Set the edge color
    p3 = patch(animPlot,'Faces', s3.F, 'Vertices', s3.V);
    set(p3, 'facec', 'flat');                  % Set the face color flat
    set(p3, 'FaceColor', s3.C);           % Set the color (from file)
    set(p3, 'EdgeColor', 'none');              % Set the edge color
    p4 = patch(animPlot,'Faces', s4.F, 'Vertices', s4.V);
    set(p4, 'facec', 'flat');                  % Set the face color flat
    set(p4, 'FaceColor', s4.C);           % Set the color (from file)
    set(p4, 'EdgeColor', 'none');              % Set the edge color
    p5 = patch(animPlot,'Faces', s5.F, 'Vertices', s5.V);
    set(p5, 'facec', 'flat');                  % Set the face color flat
    set(p5, 'FaceColor', s5.C);           % Set the color (from file)
    set(p5, 'EdgeColor', 'none');              % Set the edge color
    p6 = patch(animPlot,'Faces', s6.F, 'Vertices', s6.V);
    set(p6, 'facec', 'flat');                  % Set the face color flat
    set(p6, 'FaceColor', s6.C);           % Set the color (from file)
    set(p6, 'EdgeColor', 'none');              % Set the edge color
        
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

        hold(animPlot,'on')
        
        TI_0_1	= eye(4);

        TFbkI_h_1 = zeros(4,4,n+1);

        TFbkI_h_1(:,:,1) = TI_0_1;

        for i = 1:n

            Rot_x_h = [	1	0               0                   0
                        0	cos(alpha_i(i))	-sin(alpha_i(i))	0
                        0	sin(alpha_i(i))	cos(alpha_i(i))     0
                        0	0               0                   1 ];

            Trn_x_h = [	1 0 0 a_i(i)
                        0 1 0 0
                        0 0 1 0
                        0 0 0 1     ];

            Rot_z_i = [	cos(thetaPlus_i(i)+q_posFbk_1(i,t))	-sin(thetaPlus_i(i)+q_posFbk_1(i,t))	0	0
                        sin(thetaPlus_i(i)+q_posFbk_1(i,t))	cos(thetaPlus_i(i)+q_posFbk_1(i,t))     0	0
                        0               0                   1	0
                        0               0                   0	1 ];

            Trn_z_i = [	1 0 0 0
                        0 1 0 0
                        0 0 1 d_i(i)
                        0 0 0 1     ];

            TFbkI_h_1(:,:,i+1) = TFbkI_h_1(:,:,i) * Rot_x_h * Trn_x_h * Rot_z_i * Trn_z_i;
        end

        nv0 = [s0.V,ones(size(s0.V(:,1)))]*TFbkI_h_1(:,:,1)';
        nv1 = [s1.V,ones(size(s1.V(:,1)))]*TFbkI_h_1(:,:,2)';
        nv2 = [s2.V,ones(size(s2.V(:,1)))]*TFbkI_h_1(:,:,3)';
        nv3 = [s3.V,ones(size(s3.V(:,1)))]*TFbkI_h_1(:,:,4)';
        nv4 = [s4.V,ones(size(s4.V(:,1)))]*TFbkI_h_1(:,:,5)';
        nv5 = [s5.V,ones(size(s5.V(:,1)))]*TFbkI_h_1(:,:,6)';
        nv6 = [s6.V,ones(size(s6.V(:,1)))]*TFbkI_h_1(:,:,7)';

        set(p0,'Vertices',nv0(:,1:3))
        set(p1,'Vertices',nv1(:,1:3))
        set(p2,'Vertices',nv2(:,1:3))
        set(p3,'Vertices',nv3(:,1:3))
        set(p4,'Vertices',nv4(:,1:3))
        set(p5,'Vertices',nv5(:,1:3))
        set(p6,'Vertices',nv6(:,1:3))
            
        hold(animPlot,'off')

        set(animPlot,	'Projection','perspective', ...
                        'PlotBoxAspectRatio',[1 1 1], ...
                        'DataAspectRatio',[1 1 1]);
        drawnow;
    end

end