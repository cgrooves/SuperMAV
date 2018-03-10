% load parameters in
A = csvread('lhc_5000.csv',1,0);

kp_chi = A(:,1);
ki_chi = A(:,2);

kp_phi = A(:,3);
kd_phi = A(:,4);
ki_phi = A(:,5);

n = size(A,1);

t_rise = zeros(n,1);
t_set = zeros(n,1);
overshoot = zeros(n,1);

% pass in each row to model
for i = 1:n
    [t_rise(i), t_set(i), overshoot(i)] = course_response(...
        kp_chi(i),ki_chi(i),kp_phi(i),kd_phi(i),ki_phi(i),0);
    disp(i)
end

% export table
T = table(kp_chi, ki_chi, kp_phi, kd_phi, ki_phi, t_rise, t_set, overshoot);
writetable(T,'LHC_5000_output.csv');