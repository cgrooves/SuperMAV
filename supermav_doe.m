% File for running a DOE for the SuperMAV gain-finding project

% Load Inputs
% Inputs are assumed to be formatted in this manner:
% kp_chi, ki_chi, kp_phi, kd_phi, ki_phi, kp_h, ki_h, kp_theta, kd_theta,
% ki_theta, kp_V, ki_V
headers = {'kp_chi','ki_chi','kp_phi','kd_phi','kp_h','ki_h','kp_theta',...
    'kd_theta','ki_theta','kp_V','ki_V'};

doe_inputs = csvread('Inputs/DOE10inputs.csv',1,0);

% Separate out the inputs
chi_gains = doe_inputs(:,1:2);

phi_gains = doe_inputs(:,3:5);

h_gains = doe_inputs(:,6:7);

theta_gains = doe_inputs(:,8:10);

V_gains = doe_inputs(:,11:12);

% Get size of doe
n = size(doe_inputs,1);

% Allocate output vectors
chi_out = zeros(n,3);
h_out = zeros(n,3);
p_out = zeros(n,1);
q_out = zeros(n,1);

%% RUN THE DOE
for i = 1:n
    [S_chi, S_h, ts_p, ts_q] = mav_response(chi_gains(i,:), phi_gains(i,:),...
        h_gains(i,:), theta_gains(i,:), V_gains(i,:), 0);
    
    chi_out(i,1) = S_chi.RiseTime;
    chi_out(i,2) = S_chi.SettlingTime;
    chi_out(i,3) = S_chi.Overshoot;
    
    h_out(i,1) = S_h.RiseTime;
    h_out(i,2) = S_h.SettlingTime;
    h_out(i,3) = S_h.Overshoot;
    
    p_out(i) = ts_p;
    q_out(i) = ts_q;
end

%% Write out Data

T = table(chi_gains, phi_gains, h_gains, theta_gains, V_gains,...
    chi_out, h_out, p_out, q_out);

writetable(T,'Outputs/supermav_doe_output001.csv');