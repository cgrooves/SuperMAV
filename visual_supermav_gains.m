function out = visual_supermav_gains(filename, line, testing)

% Plot the output of chi, h, and p and q for a particular set of gains
% found on the line of filename (a .csv file, with gains ordered in sets of
% kp, kd, ki for: chi, phi, h, theta, V.

% manual gain values
if testing == 1
kp_chi = 1.2;
ki_chi = 0.041;

kp_phi = 0.88;
kd_phi = 0.16;
ki_phi = 0.304;

kp_h = 0.0181;
kd_h = 0.005;
ki_h = 0.004;

kp_theta = -1.5;
kd_theta = -0.4;
ki_theta = -.5 ;

kp_V = 0.234;
ki_V = 0.52;

gains = [kp_phi kd_phi ki_phi kp_chi ki_chi kp_theta kd_theta ki_theta kp_h kd_h ki_h kp_V ki_V];
else
% Read the file and get the gains
gains = csvread(filename,line-1,0,[line-1, 0, line-1, 12]);
end

% feed the gains into the sim
out = mav_response(gains(4:5),gains(1:3),gains(9:11),gains(6:8),gains(12:13),1);

end