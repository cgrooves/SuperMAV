% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%

function out = forces_moments(x, delta, wind, P)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
    % compute wind in the vehicle frame
    R_roll = [...
          1, 0, 0;...
          0, cos(phi), sin(phi);...
          0, -sin(phi), cos(phi)];
    R_pitch = [...
          cos(theta), 0, -sin(theta);...
          0, 1, 0;...
          sin(theta), 0, cos(theta)];
    R_yaw = [...
          cos(psi), sin(psi), 0;...
          -sin(psi), cos(psi), 0;...
          0, 0, 1];
      
    % R is vehicle to body
    R = R_roll*R_pitch*R_yaw;
    
    
    windGust = wind(4:6)' * R';
  
    % compute wind data in NED
    w_n = w_ns + windGust(1);
    w_e = w_es + windGust(2);
    w_d = w_ds + windGust(3);
    
    windBody = [w_n w_e w_d] * R;
    
    % compute air data
    Vab = [u - windBody(1); v - windBody(2); w - windBody(3)];
    Va = sqrt(Vab(1)^2 + Vab(2)^2 + Vab(3)^2);
    alpha = atan(Vab(3)/Vab(1));
    beta = asin(Vab(2)/sqrt(Vab(1)^2 + Vab(2)^2 + Vab(3)^2));
    Clo = P.C_L_0;         
    Cla = P.C_L_alpha;     
    Clq = P.C_L_q;       
    Clde = P.C_L_delta_e; 
    Cdo = P.C_D_0;       
    Cda = P.C_D_alpha;    
    Cdp = P.C_D_p;       
    Cdq = P.C_D_q;       
    Cdde = P.C_D_delta_e;  
    Cmo = P.C_m_0 ;       
    Cma = P.C_m_alpha   ;  
    Cmq = P.C_m_q;         
    Cmde = P.C_m_delta_e;  
    Cyo = P.C_Y_0;        
    Cyb = P.C_Y_beta; 
    Cyp = P.C_Y_p;    
    Cyr = P.C_Y_r;   
    Cyda = P.C_Y_delta_a; 
    Cydr = P.C_Y_delta_r; 
    Celo = P.C_ell_0;   
    Celb = P.C_ell_beta;  
    Celp = P.C_ell_p;    
    Celr = P.C_ell_r;   
    Celda = P.C_ell_delta_a;
    Celdr = P.C_ell_delta_r;
    Cno = P.C_n_0 ;    
    Cnb = P.C_n_beta;   
    Cnp = P.C_n_p;       
    Cnr = P.C_n_r;        
    Cnda = P.C_n_delta_a;  
    Cndr = P.C_n_delta_r;        
    M = P.M;           
    e = P.epsilon;       
    alpha0 = P.alpha0;
    
    m = P.mass;
    g = P.gravity;
    density = P.density;
    S = P.S;
    c = P.c;
    Sprop = P.S_prop;
    Cprop = P.C_prop;
    Kmotor = P.k_motor;
    b = P.b;
    
    % Static Coefficients
    
    

   

    
    % C coefficients
    Cl = Clo + Cla*alpha;
    Cd = Cdo + Cda*alpha;
    Cx = -Cd * cos(alpha) + Cl*sin(alpha);
    Cxq = -Cdq * cos(alpha) + Clq * sin(alpha);
    Cxde = -Cdde * cos(alpha) + Clde * sin(alpha);
    Cz = -Cd * sin(alpha) - Cl * cos(alpha);
    Czq = -Cdq * sin(alpha) - Clq * cos(alpha);
    Czde = -Cdde * sin(alpha) - Clde * cos(alpha);
    
    % compute external forces and torques on aircraft
    Force(1) =  -m*g*sin(theta) + 1/2*density*Va^2*S * (Cx + Cxq * c/(2*Va) * q + Cxde * delta_e) + 1/2*density*Sprop * Cprop * ((Kmotor*delta_t)^2-Va^2);
    Force(2) =  m*g*cos(theta)*sin(phi) + 1/2*density*Va^2*S*(Cyo + Cyb*beta + Cyp*b/(2*Va) * p + Cyr * b/(2*Va) * r + Cyda*delta_a + Cydr*delta_r);
    Force(3) =  m*g*cos(theta)*cos(phi) + 1/2*density*Va^2*S*(Cz + Czq*c/(2*Va)*q + Czde*delta_e);
    
    Torque(1) = 1/2*density*Va^2*S*(b*(Celo +Celb*beta + Celp * b/(2*Va)*p + Celr * b/(2*Va)*r + Celda*delta_a + Celdr * delta_r));
    Torque(2) = 1/2*density*Va^2*S*(c*(Cmo + Cma * alpha + Cmq * c/(2*Va) * q + Cmde * delta_e));   
    Torque(3) = 1/2*density*Va^2*S*(b*(Cno + Cnb * beta + Cnp * b/(2*Va) * p + Cnr*b/(2*Va) * r + Cnda * delta_a + Cndr * delta_r));
   
    out = [Force'; Torque'; Va; alpha; beta; w_n; w_e; w_d];
end



