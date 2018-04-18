function out = mav_response(chi_gains, phi_gains,...
    h_gains, theta_gains, V_gains, plot_flag)

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
P.kd_h = h_gains(2);
P.ki_h = h_gains(3);
% Airspeed hold using throttle
P.kp_V = V_gains(1);
P.ki_V = V_gains(2);

% initial conditions
x0 = [P.pn0, P.pe0, P.pd0, P.Va0, P.v0, P.w0, P.phi0, P.theta0, P.psi0,...
    P.p0, P.q0, P.r0];

% solve nonlinear dynamics with gains
[t,y] = ode45(@(t,y) mav_dynamics(t,y,P),[0,50],x0);

chi = y(:,9);
h = -y(:,3);
p = y(:,10);
q = y(:,11);

% Get airspeed
u = y(:,4);
v = y(:,5);
w = y(:,6);

Va = sqrt(u.^2 + v.^2 + w.^2);

% Plot course and altitude responses
if plot_flag == 1
    figure(1)
    plot(t,chi*180/pi)
    title('Course')
    figure(2)
    plot(t,h)
    title('Altitude')
    figure(3)
    plot(t,p,t,q)
    title('Roll and pitch rates')
    legend('p','q')
end
    
% get stepinfo
S_chi = stepinfo(chi,t,30*pi/180);
S_h = stepinfo(h,t,115,'SettlingTimeThreshold',0.2);
S_p = stepinfo(y(:,10),t,0,'SettlingTimeThreshold',0.1);
S_q = stepinfo(y(:,11),t,0,'SettlingTimeThreshold',0.1);

[chi_tr, chi_ts, chi_over] = get_stepinfo(S_chi);
[h_tr, h_ts, h_over] = get_stepinfo(S_h);
[~,ts_p,~] = get_stepinfo(S_p);
[~,ts_q,~] = get_stepinfo(S_q);

out = [chi_tr, chi_ts, chi_over, h_tr, h_ts, h_over, ts_p, ts_q];

end

% Takes a stepinfo Structure and gets the rise time (tr), settling time
% (ts), and overshoot (over). If tr or ts is nan, returns value of 100.
function [tr, ts, over] = get_stepinfo(SS)

tr = SS.RiseTime;

if isnan(tr)
    tr = 100;
end

ts = SS.SettlingTime;

if isnan(ts)
    ts = 100;
end

over = SS.Overshoot;

end