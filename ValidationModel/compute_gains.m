%% Enter filename, line number to read in gains from
filename = '../Outputs/sfud_50_002.csv';
line = 6;
gains = csvread(filename,line-1,0,[line-1, 0, line-1, 12]);

P.kp_phi = gains(1);
P.kd_phi = gains(2);
P.ki_phi = gains(3);

P.kp_chi = gains(4);
P.ki_chi = gains(5);

P.kp_theta = gains(6);
P.kd_theta = gains(7);
P.ki_theta = gains(8);

P.kp_h = gains(9);
P.kd_h = gains(10);
P.ki_h = gains(11);

P.kp_V = gains(12);
P.ki_V = gains(13);
