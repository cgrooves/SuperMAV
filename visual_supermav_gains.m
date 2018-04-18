function out = visual_supermav_gains(filename, line)

% Plot the output of chi, h, and p and q for a particular set of gains
% found on the line of filename (a .csv file, with gains ordered in sets of
% kp, kd, ki for: chi, phi, h, theta, V.

% Read the file and get the gains
gains = csvread(filename,line-1,0,[line-1, 0, line-1, 12]);

% feed the gains into the sim
out = mav_response(gains(1:3),gains(4:5),gains(6:8),gains(9:11),gains(12:13),1);

end