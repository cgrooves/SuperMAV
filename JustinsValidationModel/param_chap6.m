% use these revised values for the Aerosonde UAV

% % P.mass = 25;
% % % initial airspeed
% % P.Va0 = 35;        % m/s (~85 mph)
% % 
% % % autopilot sample rate
% % P.Ts = 0.01;
% % 
% % P.gravity = 9.8;
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Params for Aersonade UAV
%physical parameters of airframe
P.mass = 25;
P.Jx   = 0.8244;
P.Jy   = 1.135;
P.Jz   = 1.759;
P.Jxz  = .1204;
P.density = 1.2682;
P.gravity = 9.8;

Jx = P.Jx;
Jy = P.Jy;
Jz = P.Jz;
Jxz = P.Jxz;
% aerodynamic coefficients
P.S_wing        = 0.55;
P.b             = 2.8956;
P.c             = 0.18994;
P.S_prop        = 0.2027;
P.rho           = 1.2682;
P.k_motor       = 80;
P.k_T_P         = 0;
P.k_Omega       = 0;
P.e             = 0.9;
P.S             = 0.2589;

P.C_L_0         = 0.28;
P.C_L_alpha     = 3.45;
P.C_L_q         = 0.0;
P.C_L_delta_e   = -0.36;
P.C_D_0         = 0.03;
P.C_D_alpha     = 0.30;
P.C_D_p         = 0.0437;
P.C_D_q         = 0.0;
P.C_D_delta_e   = 0.0;
P.C_m_0         = -0.02338;
P.C_m_alpha     = -0.38;
P.C_m_q         = -3.6;
P.C_m_delta_e   = -0.5;
P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.98;
P.C_Y_p         = 0.0;
P.C_Y_r         = 0.0;
P.C_Y_delta_a   = 0.0;
P.C_Y_delta_r   = -0.17;
P.C_ell_0       = 0.0;
P.C_ell_beta    = -0.12;
P.C_ell_p       = -0.26;
P.C_ell_r       = 0.14;
P.C_ell_delta_a = 0.08;
P.C_ell_delta_r = 0.105;
P.C_n_0         = 0.0;
P.C_n_beta      = 0.25;
P.C_n_p         = 0.022;
P.C_n_r         = -0.35;
P.C_n_delta_a   = 0.06;
P.C_n_delta_r   = -0.032;
P.C_prop        = 1.0;
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;

% wind parameters
P.wind_n = 0;%3;
P.wind_e = 0;%2;
P.wind_d = 0;
P.L_u = 200;
P.L_v = 200;
P.L_w = 50;
P.sigma_u = 1.06; 
P.sigma_v = 1.06;
P.sigma_w = .7;

% compute gammas
P.T = Jx * Jz - Jxz^2;
T = P.T;
P.T1 = Jxz * (Jx - Jy + Jz)/T;
P.T2 = (Jz*(Jz-Jy) + Jxz^2)/T;
P.T3 = Jz/T;
P.T4 = Jxz/T;
P.T5 = (Jz - Jx)/Jy;
P.T6 = Jxz/Jy;
P.T7 = ((Jx - Jy)*Jx + Jxz^2)/T;
P.T8 = Jx/T;


% compute trim conditions using 'mavsim_chap5_trim.slx'
% initial airspeed
P.Va0 = 35;
gamma = 0*pi/180;  % desired flight path angle (radians)
R     = Inf;         % desired radius (m) - use (+) for right handed orbit, 

% autopilot sample rate
P.Ts = 0.01;

% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = P.Va0; % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate

                    %                          (-) for left handed orbit

% run trim commands
[x_trim, u_trim]=compute_trim('mavsim_trim',P.Va0,gamma,R);
P.u_trim = u_trim;
P.x_trim = x_trim;

% set initial conditions to trim conditions
% initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = x_trim(4);  % initial velocity along body x-axis
P.v0     = x_trim(5);  % initial velocity along body y-axis
P.w0     = x_trim(6);  % initial velocity along body z-axis
P.phi0   = x_trim(7);  % initial roll angle
P.theta0 = x_trim(8);  % initial pitch angle
P.psi0   = x_trim(9);  % initial yaw angle
P.p0     = x_trim(10);  % initial body frame roll rate
P.q0     = x_trim(11);  % initial body frame pitch rate
P.r0     = x_trim(12);  % initial body frame yaw rate

%prep for transfer function

% Gammas
T = P.T;
T1 = P.T1;
T2 = P.T2;
T3 = P.T3;
T4 = P.T4;
T5 = P.T5;
T6 = P.T6;
T7 = P.T7;
T8 = P.T8;

alpha = atan(P.w0/P.u0);
Cpo = T3*P.C_ell_0 + T4*P.C_n_0;
Cpb = T3*P.C_ell_beta + T4*P.C_n_beta;
Cpp = T3*P.C_ell_p + T4*P.C_n_p;
Cpr = T3*P.C_ell_r + T4*P.C_n_r;
Cpda = T3*P.C_ell_delta_a + T4*P.C_n_delta_a;
Cpdr = T3*P.C_ell_delta_r + T4*P.C_n_delta_r;
Cro = T4*P.C_ell_0 + T8*P.C_n_0;
Crb = T4*P.C_ell_beta + T8*P.C_n_beta;
Crp = T4*P.C_ell_p + T8*P.C_n_p;
Crr = T4*P.C_ell_r + T8*P.C_n_r;
Crda = T4*P.C_ell_delta_a + T8*P.C_n_delta_a;
Crdr = T4*P.C_ell_delta_r + T8*P.C_n_delta_r;

    
P.a_phi1 = -1/2 * P.density * P.Va0^2*P.S*P.b*Cpp*P.b/(2*P.Va0);
P.a_phi2 = 1/2 * P.density * P.Va0^2*P.S*P.b*Cpda;
P.a_beta1 = -1/2 * P.density * P.Va0*P.S/P.mass*P.C_Y_beta;
P.a_beta2 = 1/2 * P.density * P.Va0*P.S/P.mass *P.C_Y_delta_r;
P.a_theta1 = -1/2 * P.density * P.Va0^2*P.S*P.c/P.Jy * P.C_m_q*P.c/(2*P.Va0);
P.a_theta2 = -1/2 * P.density * P.Va0^2*P.S*P.c/P.Jy * P.C_m_alpha;
P.a_theta3 = 1/2 * P.density * P.Va0^2*P.S*P.c/P.Jy * P.C_m_delta_e;
P.a_V1 = P.density * P.Va0 * P.S/P.mass * (P.C_D_0 + P.C_D_alpha * alpha + P.C_D_delta_e * u_trim(1)) + P.density * P.S_prop * P.C_prop * P.Va0/P.mass;
P.a_V2 = P.density * P.S_prop/P.mass * P.C_prop * P.k_motor^2 * u_trim(4);
P.a_V3 = P.gravity * cos(x_trim(8) - x_trim(9));


% compute different transfer functions
[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
    = compute_tf_model(x_trim,u_trim,P);

% linearize the equations of motion around trim conditions
[A_lon, B_lon, A_lat, B_lat] = compute_ss_model('mavsim_trim',x_trim,u_trim);

% altitude hold zones
P.altitude_take_off_zone = 50;   
P.altitude_hold_zone = 75;
            

% gain constants
delta_r_max = 25 * pi/180;
e_beta_max = 10 * pi/180;

P.delta_r_max = delta_r_max;
P.e_beta_max = e_beta_max;

delta_a_max = 10 * pi/180;
e_phi_max = 15 * pi/180;

P.delta_a_max = delta_a_max;
P.e_phi_max = e_phi_max;

delta_e_max = 45 * pi/180;
e_theta_max = 10 * pi/180;

P.delta_e_max = delta_e_max;
P.e_theta_max = e_theta_max;

P.theta_max = 30 * pi/180;
P.theta_takeOff = 30 * pi/180;
P.t_max = 1;



Va_nom = 10;

%zeta values
zetaPhi = 1;
zetaTheta = 0.707;
zetaH = 1;
zetax = 0.707;
zetaV2 = 0.707;
zetaV = 0.707;
zetaBeta = 1;

% successive loop multipliers
Wh = 30;
Wx = 10;
Wv2 = 10;


% side slip gains
P.Kp_beta = P.delta_r_max/P.e_beta_max * sign(P.a_beta2);
P.Ki_beta = 1/P.a_beta2 * ((P.a_beta1 + P.a_beta2 * P.Kp_beta)/(zetaBeta*2))^2;

% phi gains
P.Kp_phi = delta_a_max/e_phi_max * sign(P.a_phi2);
P.wn_phi = sqrt(abs(P.a_phi2) * delta_a_max/e_phi_max);
P.Kd_phi = (2*zetaPhi * P.wn_phi - P.a_phi1)/P.a_phi2;
P.Ki_phi = 0;

% a1 = P.a_phi2*P.Kd_phi;
% a2 = 1;
% a3 = P.a_phi1 + P.a_phi2 * P.Kd_phi;
% a4 = P.a_phi2 * P.Kp_phi;
% h = tf(a1,[a2, a3, a4]);
% rlocus(h)

% theta gains
P.Kp_theta = delta_e_max/e_theta_max * sign(P.a_theta3);
P.wn_theta = sqrt(P.a_theta2 + delta_e_max/e_theta_max * abs(P.a_theta3));
P.Kd_theta = (2*zetaTheta * P.wn_theta - P.a_theta1)/P.a_theta3;
Ktheta_DC = P.Kp_theta * P.a_theta3/(P.a_theta2 + P.Kp_theta * P.a_theta3);
P.Ki_theta = .5;%.5;

% altitude gains
P.wn_h = 1/Wh * P.wn_theta;
P.Ki_h = P.wn_h^2/(Ktheta_DC * P.Va0);
P.Kp_h = 2*zetaH * P.wn_h/(Ktheta_DC * P.Va0);

% heading gains
P.wn_x = 1/Wx * P.wn_phi;
P.Kp_x = 2*zetax * P.wn_x * P.Va0/P.gravity;
P.Ki_x = P.wn_x^2 * P.Va0/P.gravity;

% airspeed using pitch angle
P.wn_v2 = 1/Wv2 * P.wn_theta;
P.Ki_v2 = -P.wn_v2^2/(Ktheta_DC * P.gravity);
P.Kp_v2 = (P.a_V1 - 2 * zetaV2 * P.wn_v2)/(Ktheta_DC * P.gravity);

% throttle gains
P.wn_v = P.wn_v2;
P.Ki_v = P.wn_v^2/P.a_V2;
P.Kp_v = (2 * zetaV * P.wn_v - P.a_V1)/P.a_V2;








