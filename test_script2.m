close; clear; clc;
% Caleb's control gains
chi = [0.88, .041];
phi = [1.5, .16, .304];


[S_chi, S_h, ts_p, ts_q] = course_response([0.88,.041],[.88,.16,.304],[-1.6,-.4338,0],[.0181,.0039],[.234,.52],1)

% Justin's control gains
% [S_chi, S_h, ts_p, ts_q] = course_response([3.1942,1.4288],[.667,.0837,0],[-4.5,-.771,0.5],[.0209,.0033],[.0289,.0033],1)