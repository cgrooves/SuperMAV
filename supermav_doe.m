% File for running a DOE for the SuperMAV gain-finding project

% Load Inputs
% Inputs are assumed to be formatted in this manner:
% kp_phi, kd_phi, ki_phi, kp_chi, ki_chi, kp_theta, kd_theta,
% ki_theta, kp_h, kd_h, ki_h, kp_V, ki_V


doe_inputs = xlsread('Inputs/Space Filling Latin Hypercube 50 inputs PlusMinus_kdh.xlsx');

% Separate out the inputs
phi_gains = doe_inputs(:,1:3);

chi_gains = doe_inputs(:,4:5);

theta_gains = doe_inputs(:,6:8);

h_gains = doe_inputs(:,9:11);

% theta_gains(:,3) = -1*theta_gains(:,3);

V_gains = doe_inputs(:,12:13);

% Get size of doe
n = size(doe_inputs,1);

% Allocate output vectors
chi_out = zeros(n,3);
h_out = zeros(n,3);
p_out = zeros(n,1);
q_out = zeros(n,1);

%% RUN THE DOE
for i = 1:50
    
    out = mav_response(chi_gains(i,:), phi_gains(i,:), h_gains(i,:), ...
    theta_gains(i,:), V_gains(i,:), 0);

    chi_out(i,:) = out(1:3);
    h_out(i,:) = out(4:6);
    p_out(i) = out(7);
    q_out(i) = out(8);
    
    i
    
end

%% Write out Data

T = table(phi_gains, chi_gains, theta_gains, h_gains, V_gains,...
    chi_out, h_out, p_out, q_out);

writetable(T,'Outputs/supermav_doe_output_130_kdh02.csv');