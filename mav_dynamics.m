function xdot = mav_dynamics(t,x,P,chi_c)

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

% intialize control surface inputs
persistent delta_e;
persistent delta_a;
persistent delta_r;
persistent delta_t;

flag = 0;
if t == 0
    flag = 1;
end

% Autopilot
delta_e = -0.103;
delta_r = 0.0;
delta_t = 0.466;

phi_c = course_hold(chi_c,psi,flag,P);
delta_a = roll_hold(phi_c,phi,p,flag,P);

% get forces and moments
uu = forces_moments(x,[delta_e, delta_a, delta_r, delta_t],[0 0 0 0 0 0],P);

fX = uu(1);
fY = uu(2);
fZ = uu(3);
ell = uu(4);
m = uu(5);
n = uu(6);

% Equations of Motion
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

xdot = [...
    pn_dot; pe_dot; pd_dot; ...
    u_dot; v_dot; w_dot; ...
    phi_dot; theta_dot; psi_dot; ...
    p_dot; q_dot; r_dot]; % assign derivatives of states to xdot
end

function phi_c = course_hold(chi_c, chi, flag, P)
    persistent chi_integrator;
    persistent chi_error_d1;
    
    if flag == 1
        chi_integrator = 0;
        chi_error_d1 = 0;
    end
    
    error = chi_c - chi;
    
    % discrete integrator
    chi_integrator = chi_integrator + P.Ts/2*(error + chi_error_d1);
    
    % compute output command and saturate
    phi_c = P.kp_chi*error + P.ki_chi*chi_integrator;
    
    chi_error_d1 = error; % store old error value
    
%     % integrator anti-windup
    if P.ki_chi ~= 0
        u_unsat = P.kp_chi*error + P.ki_chi*chi_integrator;
        chi_integrator = chi_integrator + P.Ts/P.ki_chi*(phi_c - u_unsat);
    end
end

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
    delta_a = sat(P.kp_phi*error + P.ki_phi*phi_integrator - P.kd_phi*p...
        ,P.delta_a_up,P.delta_a_down);
    
    phi_error_d1 = error; % store old error value
    
    % integrator anti-windup
    if P.ki_phi ~= 0
        u_unsat = P.kp_phi*error + P.ki_phi*phi_integrator + P.kd_phi*p;
        phi_integrator = phi_integrator + P.Ts/P.ki_phi*(delta_a - u_unsat);
    end
end

function out = sat(in, up_limit, low_limit)
  if in > up_limit
      out = up_limit;
  elseif in < low_limit
      out = low_limit;
  else
      out = in;
  end
end