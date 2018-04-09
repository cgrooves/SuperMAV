close; clear; clc;
% Caleb's control gains
[S_chi, S_h, ts_p, ts_q] = course_response([1.2,.041],[0.88,.16,.304],[-1.5,-.4,0],[.0181,.004],[.234,.52],1)

% Justin's control gains
% [S_chi, S_h, ts_p, ts_q] = course_response([3.1942,1.4288],[.667,.0837,0],[-4.5,-.771,0.5],[.0209,.0033],[.0289,.0033],1)