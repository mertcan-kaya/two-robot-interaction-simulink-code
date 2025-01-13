function qi = getInitialPoints(p_ei,r_ei,robot_model,ee_att,ea_set,config)

    R0_Ei = getRotMatfromEA(r_ei,ea_set);
    
    T0_Ei = [R0_Ei, p_ei;zeros(1,3) 1];
    
    [a_i,alpha_i,d_i,thetaPlus_i] = kinematicParametersOrigin(robot_model);

	d_e = getEElength(ee_att);

    q_ref = zeros(6,1);

    qi = invGeo(T0_Ei,a_i,alpha_i,d_i,d_e,thetaPlus_i,q_ref,config);

end