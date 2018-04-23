function xdot = mav_dynamics(t,x,P)
%% Initialization
% Make the state something a little more readable
pn = x(1);
pe = x(2);
pd = x(3);
u = x(4);
v = x(5);
w = x(6);
phi = x(7);
theta = x(8);
psi = x(9);
p = x(10);
q = x(11);
r = x(12);

h = -pd;

% intialize control surface inputs
persistent delta_e;
persistent delta_a;
persistent delta_r;
persistent delta_t;
persistent Va;

flag = 0;

if t == 0
    flag = 1;
    Va = P.Va0;
end

%% Autopilot Commands
% Get commanded inputs
Va_c = 35;
[chi_c, h_c] = benchmark_input(t);

% display(chi_c);
% display(h_c);
% display(t);

% Autopilot - Lateral
delta_r = 0.0; % hold rudder steady

phi_c = course_hold(chi_c,psi, r, flag,P);

delta_a = roll_hold(phi_c,phi,p,flag,P);

% Autopilot - Longitudinal (using altitude hold controllers)
delta_t = airspeed_throttle_hold(Va_c, Va, flag, P);

% Calculate hdot
hdot = -(-sin(theta)*u + sin(phi)*cos(theta)*v + cos(phi)*cos(theta)*w);
% hdot = 0;

theta_c = altitude_hold(h_c, h, hdot, flag, P);
delta_e = pitch_hold(theta_c, theta, q, flag, P);

%% Forces and Moments
% get forces and moments
uu = forces_moments(x,[delta_e, delta_a, delta_r, delta_t],[0 0 0 0 0 0],P);

fX = uu(1);
fY = uu(2);
fZ = uu(3);
ell = uu(4);
m = uu(5);
n = uu(6);
Va = uu(7);

%% Dynamics - Equations of Motion
%* Checked
pn_dot = cos(theta)*cos(psi)*u + (sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi))*v + (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi))*w;
pe_dot = cos(theta)*sin(psi)*u + (sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi))*v + (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi))*w;
pd_dot = -sin(theta)*u + sin(phi)*cos(theta)*v + cos(phi)*cos(theta)*w;

%* Checked
u_dot = r*v - q*w + fX/P.mass;
v_dot = p*w - r*u + fY/P.mass;
w_dot = q*u - p*v + fZ/P.mass;

%* Checked
phi_dot = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
theta_dot = cos(phi)*q - sin(phi)*r;
psi_dot = sin(phi)*q/cos(theta) + cos(phi)*r/cos(theta);

%* Checked
p_dot = P.Gamma1*p*q - P.Gamma2*q*r + P.Gamma3*ell + P.Gamma4*n;
q_dot = P.Gamma5*p*r - P.Gamma6*(p^2 - r^2) + m/P.Jy;
r_dot = P.Gamma7*p*q - P.Gamma1*q*r + P.Gamma4*ell + P.Gamma8*n;

xdot = [pn_dot; pe_dot; pd_dot; u_dot; v_dot; w_dot; phi_dot; theta_dot; psi_dot; p_dot; q_dot; r_dot]; % assign derivatives of states to xdot
end

%% Autopilot Controllers

% COURSE HOLD****************************************************
function phi_c = course_hold(chi_c, chi, r, flag, P)
    persistent chi_integrator;
    persistent chi_error_d1;
    
    if flag == 1
        chi_integrator = 0;
    end
    
    error = chi_c - chi;

    % compute output command and saturate
    u_unsat = P.kp_chi*error + P.ki_chi*chi_integrator;
    phi_c = sat(u_unsat, 15*pi/180, -15*pi/180);
    
    chi_error_d1 = error; % store old error value
    
    % integrator anti-windup
    if r < (1*pi/180)
        chi_integrator = chi_integrator + P.Ts/2*(error + chi_error_d1);
    end
end

% ROLL HOLD**********************************************
function delta_a = roll_hold(phi_c, phi, p, flag, P)
    persistent phi_integrator;
    persistent phi_error_d1;
    
    if flag == 1
        phi_integrator = 0;
        phi_error_d1 = 0;
    end
    
    error = phi_c - phi;
    
    % discrete integrator
    phi_integrator = phi_integrator + P.Ts/2*(error + phi_error_d1);
    
    % compute output command and saturate
    u_unsat = P.kp_phi*error + P.ki_phi*phi_integrator - P.kd_phi*p;
    delta_a = sat(u_unsat, P.delta_a_up, P.delta_a_down);
    
    % integrator anti-windup
    if P.ki_phi ~= 0% < 5*pi/180
        phi_integrator = phi_integrator + P.Ts/P.ki_phi*(delta_a - u_unsat);
    end
    
    phi_error_d1 = error; % store old error value    

end

% AIRSPEED HOLD USING THROTTLE ********************************
function delta_t = airspeed_throttle_hold(Va_c, Va, flag, P)
    persistent Va_integrator;
    persistent Va_error_d1;
    
    if flag == 1
        Va_integrator = 0;
        Va_error_d1 = 0;
    end
    
    error = Va_c - Va;
    
    % discrete integrator
    Va_integrator = Va_integrator + P.Ts/2*(error + Va_error_d1);
    
    % compute output command and saturate
    u_unsat = 0.46 + P.kp_V*error + P.ki_V*Va_integrator;
    delta_t = sat(u_unsat,1,0);
    
    Va_error_d1 = error; % store old error value
    
    % integrator anti-windup
    if P.ki_V ~= 0
        Va_integrator = Va_integrator + P.Ts/P.ki_V*(delta_t - u_unsat);
    end

end

% ALTITUDE HOLD USING PITCH **********************************
function theta_c = altitude_hold(h_c, h, hdot, flag, P)
    persistent h_integrator;
    persistent h_error_d1;
    
    if flag == 1
        h_integrator = 0;
        h_error_d1 = 0;
    end
    
    error = h_c - h;
    
    % discrete integrator
%     h_integrator = h_integrator + P.Ts/2*(error + h_error_d1);
%     h_integrator = 0;
    
    % compute output command        
    u_unsat = P.kp_h*error + P.ki_h*h_integrator - P.kd_h*hdot;
    theta_c = sat(u_unsat, 15*pi/180, -15*pi/180);
    
    % integrator anti-windup
    if hdot < 1/100
        h_integrator = h_integrator + P.Ts/2*(error + h_error_d1);
    end    
    
    h_error_d1 = error; % store old error value    

end

% PITCH ALTITUDE HOLD************************************
function delta_e = pitch_hold(theta_c, theta, q, flag, P)

    persistent theta_integrator;
    persistent theta_integrator_d1;

    if flag == 1
        theta_integrator = 0;
        theta_integrator_d1 = 0;
    end
    
    error = theta_c - theta; % define theta error
    
    % integrated error - discrete algorithm
%     theta_integrator = theta_integrator + P.Ts/2*(error + theta_integrator_d1);

    % Saturated elevator command
    u_unsat = P.kp_theta*error - P.kd_theta*q - P.ki_theta*theta_integrator;
    delta_e = sat(u_unsat, P.delta_e_up, P.delta_e_down);
    
    % Integrator anti-windup scheme
    if q < 1*pi/180
        theta_integrator = theta_integrator + P.Ts/2*(delta_e - u_unsat);
    end

end

% SATURATION CONTROL*************************************************
function out = sat(in, up_limit, low_limit)
  if in > up_limit
      out = up_limit;
  elseif in < low_limit
      out = low_limit;
  else
      out = in;
  end
end

% Benchmark maneuver ******************************
function [chi_c, h_c] = benchmark_input(t)

h_c = 115;
chi_c = 30*pi/180;
    
end