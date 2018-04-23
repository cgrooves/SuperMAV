function [T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
    = compute_tf_model(x_trim,u_trim,P)
% x_trim is the trimmed state,
% u_trim is the trimmed input

% add stuff here
    
a_phi1 = P.a_phi1;
a_phi2 = P.a_phi2;
a_beta1 = P.a_beta1;
a_beta2 = P.a_beta2;  
a_theta1 = P.a_theta1;
a_theta2 = P.a_theta2; 
a_theta3 = P.a_theta3; 
a_V1 = P.a_V1;
a_V2 = P.a_V2;  
a_V3 = P.a_V3; 
Va_trim = P.Va0;
theta_trim = x_trim(8);
    
% define transfer functions
T_phi_delta_a   = tf([a_phi2],[1,a_phi1,0]);
T_chi_phi       = tf([P.gravity/Va_trim],[1,0]);
T_theta_delta_e = tf(a_theta3,[1,a_theta1,a_theta2]);
T_h_theta       = tf([Va_trim],[1,0]);
T_h_Va          = tf([theta_trim],[1,0]);
T_Va_delta_t    = tf([a_V2],[1,a_V1]);
T_Va_theta      = tf([-a_V3],[1,a_V1]);
T_v_delta_r     = tf([Va_trim*a_beta2],[1,a_beta1]);

