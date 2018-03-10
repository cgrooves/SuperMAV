function [t_rise, t_set, overshoot] = course_response(kp_chi, ki_chi,...
    kp_phi, kd_phi, ki_phi, plot_flag)

% get P struct
vehicle_params;

% Assign control gains to struct
P.kp_chi = kp_chi;
P.ki_chi = ki_chi;
P.kp_phi = kp_phi;
P.kd_phi = kd_phi;
P.ki_phi = ki_phi;

% input step
chi_c = 20*pi/180;

% initial conditions
x0 = [P.pn0, P.pe0, P.pd0, P.u0, P.v0, P.w0, P.phi0, P.theta0, P.psi0,...
    P.p0, P.q0, P.r0];

% solve nonlinear dynamics with gains
[t,y] = ode45(@(t,y) mav_dynamics(t,y,P,chi_c),[0,50],x0);

if plot_flag == 1
    plot(t,y(:,9))
end
    
% get stepinfo
S = stepinfo(y(:,9),t,chi_c);

t_rise = S.RiseTime;
t_set = S.SettlingTime;
overshoot = S.Overshoot;

if isnan(t_rise)
    t_rise = 100;
end

if isnan(t_set)
    t_set = 100;
end

y = y(:,9);

end