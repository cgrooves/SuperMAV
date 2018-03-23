[S_chi, S_h] = course_response([1.2,.041],[.88,.16,.304],[-1.5,-.4,-.5],[.0181,.0039],[.234,.52],1)

% t = 0:.1:50;
% chi_c = [];
% h_c = [];
% 
% for i = 1:length(t)
%     [chi_c(i) h_c(i)] = benchmark_input(t(i));
% end
% 
% plot(t,chi_c,t,h_c)