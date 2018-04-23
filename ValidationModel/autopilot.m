function y = autopilot(uu,P)
%
% autopilot for mavsim
% 
% Modification History:
%   2/11/2010 - RWB
%   5/14/2010 - RWB
%   9/30/2014 - RWB
%   

    % process inputs
    NN = 0;
%    pn       = uu(1+NN);  % inertial North position
%    pe       = uu(2+NN);  % inertial East position
    h        = uu(3+NN);  % altitude
    u       = uu(4+NN);  % airspeed
    v       = uu(5+NN);
    w       = uu(6+NN);
    Va = uu(7+NN);
    alpha = uu(8+NN);
    beta_air = uu(9+NN);
    phi      = uu(10+NN);  % roll angle
    theta    = uu(11+NN);  % pitch angle
    chi      = uu(12+NN);  % course angle
    p        = uu(13+NN); % body frame roll rate
    q        = uu(14+NN); % body frame pitch rate
    r        = uu(15+NN); % body frame yaw rate
    NN = NN+22;
    Va_c     = uu(23);  % commanded airspeed (m/s)
    h_c      = uu(24);  % commanded altitude (m)
    chi_c    = uu(25);  % commanded course (rad)
    NN = NN+3;
    t        = uu(26);   % time
    
    % Calculate
    hdot = -(-sin(theta)*u + sin(phi)*cos(theta)*v + cos(phi)*cos(theta)*w);
    
    autopilot_version = 2;
        % autopilot_version == 1 <- used for tuning
        % autopilot_version == 2 <- standard autopilot defined in book
        % autopilot_version == 3 <- Total Energy Control for longitudinal AP
    switch autopilot_version
        case 1
           [delta, x_command] = autopilot_tuning(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
        case 2
           [delta, x_command] = autopilot_uavbook(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P,hdot);
        case 3
               [delta, x_command] = autopilot_TECS(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
    end
    y = [delta; x_command];
end
    
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot versions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_tuning
%   - used to tune each loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_tuning(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)

    mode = 2;
    switch mode
        case 1 % tune the roll loop
            phi_c = chi_c; % interpret chi_c to autopilot as course command
            
            flag = 0;
            
            if t == 0 
                flag = 1;
            end
            
            delta_a = roll_hold(phi_c,phi,p,flag,P);
            
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
            delta_e = P.u_trim(1);
            delta_t = P.u_trim(4);
            theta_c = 0;
        case 2, % tune the course loop
            if t==0,
                flag = 1;
            else
                flag = 0;
            end
            
            phi_c   = course_hold(chi_c, chi, r, flag, P);
            delta_a = roll_hold(phi_c, phi, p, flag, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
            delta_e = P.u_trim(1);
            delta_t = P.u_trim(4);
            theta_c = 0;
        case 3, % tune the throttle to airspeed loop and pitch loop simultaneously
            theta_c = 20*pi/180; % + h_c;
            chi_c = 0;
            if t==0,
                
                phi_c   = course_hold(chi_c, chi, r, 1, P);
                delta_a = roll_hold(phi_c, phi, p, 1, P);

                delta_t = airspeed_throttle_hold(Va_c, Va, 1, P);
           else
                phi_c   = course_hold(chi_c, chi, r, 0, P);
                delta_a = roll_hold(phi_c, phi, p, 0, P);
                
                delta_t = airspeed_throttle_hold(Va_c, Va, 0, P);
            end
            
            delta_e = pitch_hold(theta_c, theta, q, P);
            
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
        case 4, % tune the pitch to airspeed loop 
            chi_c = 0;
            delta_t = P.u_trim(4);
            if t==0,
                phi_c   = course_hold(chi_c, chi, r, 1, P);
                theta_c = airspeed_pitch_hold(Va_c, Va, 1, P);
                delta_a = roll_hold(phi_c, phi, p, 1, P);
           else
                phi_c   = course_hold(chi_c, chi, r, 0, P);
                theta_c = airspeed_pitch_hold(Va_c, Va, 0, P);
                delta_a = roll_hold(phi_c, phi, p, 0, P);
            end
            
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
        case 5, % tune the pitch to altitude loop 
            chi_c = 0;
            if t==0,
                phi_c   = course_hold(chi_c, chi, r, 1, P);
                delta_a = roll_hold(phi_c, phi, p, 1, P);
                theta_c = altitude_hold(h_c, h, 1, P);
                delta_t = airspeed_throttle_hold(Va_c, Va, 1, P);
           else
                phi_c   = course_hold(chi_c, chi, r, 0, P);
                delta_a = roll_hold(phi_c, phi, p, 0, P);
                theta_c = altitude_hold(h_c, h, 0, P);
                delta_t = airspeed_throttle_hold(Va_c, Va, 0, P);
            end
            
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
      end
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_uavbook
%   - autopilot defined in the uavbook
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_uavbook(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P,hdot)

    %----------------------------------------------------------
    % lateral autopilot
    if t==0

        delta_r = 0; %sideslip_hold(beta_air, 1, P);
        
        phi_c   = course_hold(chi_c, chi, r, 1, P);
        delta_a = roll_hold(phi_c, phi, p, 1, P);

    else
        phi_c   = course_hold(chi_c, chi, r, 0, P);
        delta_a = roll_hold(phi_c, phi, p, 0, P);
        
        delta_r = 0; %sideslip_hold(beta_air, 0, P);
    end
  
    
    %----------------------------------------------------------
    % longitudinal autopilot
    
    % define persistent variable for state of altitude state machine
    persistent altitude_state;
    
    % initialize persistent variable
    if t==0
        h = -P.pd0;
                
        % initialize controllers
        airspeed_throttle_hold(Va_c, Va, 1, P);
        altitude_hold(h_c, h, hdot, 1, P);
        pitch_hold(0, theta, q, 1, P);
        
    end
    
    delta_t = airspeed_throttle_hold(Va_c, Va, 0, P);
    theta_c = altitude_hold(h_c, h, hdot, 0, P);

    delta_e = pitch_hold(theta_c, theta, q, 0, P);
    
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
    y = [delta; x_command];
 
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_TECS
%   - longitudinal autopilot based on total energy control systems
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_TECS(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)

    %----------------------------------------------------------
    % lateral autopilot
    if t==0,
        % assume no rudder, therefore set delta_r=0
        delta_r = 0;%coordinated_turn_hold(beta, 1, P);
        phi_c   = course_hold(chi_c, chi, r, 1, P);

    else
        phi_c   = course_hold(chi_c, chi, r, 0, P);
        delta_r = 0;%coordinated_turn_hold(beta, 0, P);
    end
    delta_a = roll_hold(phi_c, phi, p, P);     
  
    
    %----------------------------------------------------------
    % longitudinal autopilot based on total energy control
    
    
    delta_e = 0;
    delta_t = 0;
 
    
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
    y = [delta; x_command];
 
end
   


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ROLL HOLD**********************************************
function delta_a = roll_hold(phi_c, phi, p, flag, P)
    persistent phi_integrator;
    persistent phi_error_d1;
    
    if flag == 1
        phi_integrator = 0;
        phi_error_d1 = 0;
    end
    
    error = phi_c - phi;
    
    % discrete integrator
    phi_integrator = phi_integrator + P.Ts/2*(error + phi_error_d1);
    
    % compute output command and saturate
    u_unsat = P.kp_phi*error + P.ki_phi*phi_integrator - P.kd_phi*p;
    delta_a = sat(u_unsat, P.delta_a_up, P.delta_a_down);
    
    % integrator anti-windup
    if P.ki_phi ~= 0
        phi_integrator = phi_integrator + P.Ts/P.ki_phi*(delta_a - u_unsat);
    end
    
    phi_error_d1 = error; % store old error value

end

% COURSE HOLD****************************************************
function phi_c = course_hold(chi_c, chi, r, flag, P)
    persistent chi_integrator;
    persistent chi_error_d1;
    
    if flag == 1
        chi_integrator = 0;
    end
    
    error = chi_c - chi;

    % compute output command and saturate
    u_unsat = P.kp_chi*error + P.ki_chi*chi_integrator;
    phi_c = sat(u_unsat, 15*pi/180, -15*pi/180);
    
    chi_error_d1 = error; % store old error value
    
    % integrator anti-windup
    if r < (.05*pi/180)
        chi_integrator = chi_integrator + P.Ts/2*(error + chi_error_d1);
    end
end

% SLIDESLIP HOLD************************************************
function delta_r = sideslip_hold(beta_air,flag,P)
    persistent beta_integrator;
    persistent beta_error_d1;
    
    if flag == 1
        beta_integrator = 0;
        beta_error_d1 = 0;
    end
    
    % discrete integrator
    beta_integrator = beta_integrator + P.Ts/2*(beta_air + beta_error_d1);
    
    % compute output command and saturate
    delta_r = sat(-P.kp_beta*beta_air - P.ki_beta*beta_integrator, P.delta_r_up,...
        P.delta_r_down);
    
    beta_error_d1 = beta_air; % store old error value
    
    % integrator anti-windup
    if P.ki_beta ~= 0
        u_unsat = P.kp_beta*beta_air + P.ki_beta*beta_integrator;
        beta_integrator = beta_integrator + P.Ts/P.ki_beta*(delta_r - u_unsat);
    end
end

% PITCH ALTITUDE HOLD************************************
function delta_e = pitch_hold(theta_c, theta, q, flag, P)

    persistent theta_integrator;
    persistent theta_integrator_d1;

    if flag == 1
        theta_integrator = 0;
        theta_integrator_d1 = 0;
    end
    
    error = theta_c - theta; % define theta error
    
    % integrated error - discrete algorithm
    theta_integrator = theta_integrator + P.Ts/2*(error + theta_integrator_d1);

    % Saturated elevator command
    u_unsat = P.kp_theta*error - P.kd_theta*q - P.ki_theta*theta_integrator;
    delta_e = sat(u_unsat, P.delta_e_up, P.delta_e_down);
    
    % Integrator anti-windup scheme
    if P.ki_theta ~= 0
        theta_integrator = theta_integrator + P.Ts/2*(delta_e - u_unsat);
    end

end
% ALTITUDE HOLD USING PITCH **********************************
function theta_c = altitude_hold(h_c, h, hdot, flag, P)
    persistent h_integrator;
    persistent h_error_d1;
    
    if flag == 1
        h_integrator = 0;
        h_error_d1 = 0;
    end
    
    error = h_c - h;
    
    % discrete integrator
%     h_integrator = h_integrator + P.Ts/2*(error + h_error_d1);
%     h_integrator = 0;
    
    % compute output command        
    u_unsat = P.kp_h*error + P.ki_h*h_integrator - P.kd_h*hdot;
    theta_c = sat(u_unsat, 15*pi/180, -15*pi/180);
    
    % integrator anti-windup
    if hdot < 1/8
        h_integrator = h_integrator + P.Ts/2*(error + h_error_d1);
    end    
    
    h_error_d1 = error; % store old error value    

end

% AIRSPEED HOLD USING PITCH ******************************************
function theta_c = airspeed_pitch_hold(Va_c, Va, flag, P)
    persistent Va_integrator;
    persistent Va_error_d1;
    
    if flag == 1
        Va_integrator = 0;
        Va_error_d1 = 0;
    end
    
    error = Va_c - Va;
    
    % discrete integrator
    Va_integrator = Va_integrator + P.Ts/2*(error + Va_error_d1);
    
    % compute output command and saturate
    theta_c = P.kp_V2*error + P.ki_V2*Va_integrator;
    
    Va_error_d1 = error; % store old error value
    
    % integrator anti-windup
    if P.ki_V2 ~= 0
        u_unsat = P.kp_V2*error + P.ki_V2*Va_integrator;
        Va_integrator = Va_integrator + P.Ts/P.ki_V2*(theta_c - u_unsat);
    end
end

% AIRSPEED HOLD USING THROTTLE ********************************
function delta_t = airspeed_throttle_hold(Va_c, Va, flag, P)
    persistent Va_integrator;
    persistent Va_error_d1;
    
    if flag == 1
        Va_integrator = 0;
        Va_error_d1 = 0;
    end
    
    error = Va_c - Va;
    
    % discrete integrator
    Va_integrator = Va_integrator + P.Ts/2*(error + Va_error_d1);
    
    % compute output command and saturate
    u_unsat = 0.46 + P.kp_V*error + P.ki_V*Va_integrator;
    delta_t = sat(u_unsat,1,0);
    
    Va_error_d1 = error; % store old error value
    
    % integrator anti-windup
    if P.ki_V ~= 0
        Va_integrator = Va_integrator + P.Ts/P.ki_V*(delta_t - u_unsat);
    end

end



  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sat
%   - saturation function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = sat(in, up_limit, low_limit)
  if in > up_limit
      out = up_limit;
  elseif in < low_limit
      out = low_limit;
  else
      out = in;
  end
end
  
 