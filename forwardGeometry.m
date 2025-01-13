function [T0_h,Th_i] = forwardGeometry(a_i,alpha_i,d_i,theta_i)

    n = length(theta_i);

    Th_i = zeros(4,4,n);
    T0_h = zeros(4,4,n+1);

    T0_h(:,:,1) = eye(4);

    for i = 1:n        
        Rot_x_h = Rot_x(alpha_i(i));
        Trn_x_h	= Trn_x(a_i(i));
        Rot_z_i = Rot_z(theta_i(i));
        Trn_z_i = Trn_z(d_i(i));

        Th_i(:,:,i) = Rot_x_h * Trn_x_h * Rot_z_i * Trn_z_i;

        T0_h(:,:,i+1) = T0_h(:,:,i) * Th_i(:,:,i);
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