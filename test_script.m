clear; clc;

% Test gain values
kp_chi = 2.88;
ki_chi = 0.0164;

kp_phi = 2.90;
kd_phi = 0.041;
ki_phi = 2.69;

[t_rise, t_set, overshoot] = course_response(kp_chi,ki_chi,kp_phi,kd_phi,ki_phi,1)