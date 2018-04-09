function [t_rise, t_set, overshoot] = coursehold_response(kp_chi, ki_chi...
    , kp_phi, kd_phi, ki_phi)

% % Test gain values
% kp_phi = 0.375;
% ki_phi = 0.03;
% kd_phi = 0.0356;
% 
% kp_chi = 1.196;
% ki_chi = 0.124;
% % End test gains

% Define other parameters - aircraft and general
g = 9.81; % Earth gravity
rho = 1.2682;

Va = 35; % fly Aerosonde at 35 m/s
Vg = Va; % assume Vg = Va

S_wing = 0.55;
b = 2.8956;
C_p_p = -0.31672;
C_p_delta_a = 0.10305;

a_phi1 = -rho*Va*S_wing*b^2*C_p_p/4;
a_phi2 = rho*Va^2*S_wing*b*C_p_delta_a/2;

% Define transfer function for course-hold loop
sys = tf(...
    [g*a_phi2*kp_chi*kp_phi,... % s^2
    g*a_phi2*(ki_phi*kp_chi + ki_chi*kp_phi),... % s^1
    g*a_phi2*ki_chi*ki_phi],... % s^0
    ...
    [Vg,... % s^5
    (Vg*a_phi1+Vg*a_phi2*kd_phi),... % s^4
    Vg*a_phi2*kp_phi,... % s^3
    Vg*a_phi2*ki_phi + g*a_phi2*kp_chi*kp_phi,... % s^2
    g*a_phi2*ki_phi*kp_chi + g*a_phi2*ki_chi*kp_phi,... % s^1
    g*a_phi2*ki_chi*ki_phi... % s^0
    ]);

% Get system response
S = stepinfo(sys);

% Extract fields
t_rise = S.RiseTime;
t_set = S.SettlingTime;
overshoot = S.Overshoot;

end