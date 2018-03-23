function [S_chi, S_h, S_Va] = course_response(chi_gains, phi_gains,...
    theta_gains, h_gains, V_gains, plot_flag)

% get P struct
vehicle_params;

% Assign control gains to struct
% Course hold
P.kp_chi = chi_gains(1);
P.ki_chi = chi_gains(2);
% Roll hold
P.kp_phi = phi_gains(1);
P.kd_phi = phi_gains(2);
P.ki_phi = phi_gains(3);
% Pitch hold
P.kp_theta = theta_gains(1);
P.kd_theta = theta_gains(2);
P.ki_theta = theta_gains(3);
% Altitude Hold
P.kp_h = h_gains(1);
P.ki_h = h_gains(2);
% Airspeed hold using throttle
P.kp_V = V_gains(1);
P.ki_V = V_gains(2);

% initial conditions
x0 = [P.pn0, P.pe0, P.pd0, P.u0, P.v0, P.w0, P.phi0, P.theta0, P.psi0,...
    P.p0, P.q0, P.r0];

% solve nonlinear dynamics with gains
[t,y] = ode45(@(t,y) mav_dynamics(t,y,P),[0,50],x0);

chi = y(:,9);
h = -y(:,3);

u = y(:,4);
v = y(:,5);
w = y(:,6);

Va = sqrt(u.^2 + v.^2 + w.^2);

% Plot course and altitude responses
if plot_flag == 1
    figure(1)
    plot(t,chi)
    title('Course')
    figure(2)
    plot(t,h)
    title('Altitude')
    figure(3)
    plot(t,Va)
    title('Airspeed')
end
    
% get stepinfo
S_chi = stepinfo(chi,t,30*pi/180);
S_h = stepinfo(h,t,110);
S_Va = stepinfo(Va,t,35);

end