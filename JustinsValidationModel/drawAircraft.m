
function drawAircraft(uu,V,F,patchcolors)
% [pndot; pedot; pddot; udot; vdot; wdot; phidot; thetadot; psidot; pdot; qdot; rdot];

    % process inputs to function
    pn       = uu(1);       % inertial North position     
    pe       = uu(2);       % inertial East position
    pd       = uu(3);           
    u        = uu(4);       
    v        = uu(5);       
    w        = uu(6);       
    phi      = uu(7);       % roll angle         
    theta    = uu(8);       % pitch angle     
    psi      = uu(9);       % yaw angle     
    p        = uu(10);       % roll rate
    q        = uu(11);       % pitch rate     
    r        = uu(12);       % yaw rate    
    t        = uu(13);       % time

    % define persistent variables 
    persistent vehicle_handle;
    persistent Vertices
    persistent Faces
    persistent facecolors
    
    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(1), clf
        [Vertices,Faces,facecolors] = defineVehicleBody;
        vehicle_handle = drawVehicleBody(Vertices,Faces,facecolors,...
                                               pn,pe,pd,phi,theta,psi,...
                                               [],'normal');
        title('Vehicle')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32,47)  % set the vieew angle for figure
        axis([-1000,1000,-1000,1000,-1000,1000]);
        hold on
        
    % at every other time step, redraw base and rod
    else 
        drawVehicleBody(Vertices,Faces,facecolors,...
                           pn,pe,pd,phi,theta,psi,...
                           vehicle_handle);
    end
end

  
%=======================================================================
% drawVehicle
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawVehicleBody(V,F,patchcolors,...
                                     pn,pe,pd,phi,theta,psi,...
                                     handle,mode)
  V = rotate(V, phi, theta, psi);  % rotate vehicle
  V = translate(V, pn, pe, pd);  % translate vehicle
  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V = R*V;
  
  if isempty(handle),
  handle = patch('Vertices', V', 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',V','Faces',F);
%     set(figure(1),'ylim',[-10 10]);
    drawnow
  end
end

%%%%%%%%%%%%%%%%%%%%%%%
function pts=rotate(pts,phi,theta,psi)

  % define rotation matrix (right handed)
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
  R = R_roll*R_pitch*R_yaw;  
    % note that R above either leaves the vector alone or rotates
    % a vector in a left handed rotation.  We want to rotate all
    % points in a right handed rotation, so we must transpose
  R = R';

  % rotate vertices
  pts = R*pts;
  
end
% end rotateVert

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function pts = translate(pts,pn,pe,pd)

  pts = pts + repmat([pn;pe;pd],1,size(pts,2));
  
end

% end translate


%=======================================================================
% defineVehicleBody
%=======================================================================
function [V,F,facecolors] = defineVehicleBody

% Define the vertices (physical location of vertices
scale = 45;
fuse_l1  = 2*scale;
fuse_l3 = 5*scale;
fuse_l2 = 1*scale;
tail_h = 2*scale;
tailwing_l = 1*scale;
fuse_h = 2*scale;
fuse_w = 2*scale;
tailwing_w = 3*scale;
wing_w = 7*scale;
wing_l = 2*scale;

pt1 = [fuse_l1, 0, 0];
pt2 = [fuse_l2, fuse_w/2, -fuse_h/2];
pt3 = [fuse_l2, -fuse_w/2, -fuse_h/2];
pt4 = [fuse_l2, -fuse_w/2, fuse_h/2];
pt5 = [fuse_l2, fuse_w/2, fuse_h/2];
pt6 = [-fuse_l3, 0, 0];
pt7 = [0, wing_w/2, 0];
pt8 = [-wing_l, wing_w/2, 0];
pt9 = [-wing_l, -wing_w/2, 0];
pt10 = [0, -wing_w/2, 0];
pt11 = [-fuse_l3 + tailwing_l, tailwing_w/2, 0];
pt12 = [-fuse_l3, tailwing_w/2, 0];
pt13 = [-fuse_l3, -tailwing_w/2, 0];
pt14 = [-fuse_l3 + tailwing_l, -tailwing_w/2, 0];
pt15 = [-fuse_l3 + tailwing_l, 0, 0];
pt16 = [-fuse_l3, 0, -tail_h];

V = [pt1;pt2;pt3;pt4;pt5;pt6;pt7;pt8;pt9;pt10;pt11;pt12;pt13;pt14;pt15;pt16]';

F = [...
    1,2,3;...
    1,3,4;...
    1,4,5;...
    1,5,2;...
    2,6,3;...
    3,6,4;...
    4,6,5;...
    5,6,2;...
    7,8,9;...
    7,9,10;...
    11,12,13;...
    11,13,14;...
    15,6,16];


% V = [...
%     1, 0, 0;...   % pt 1
%     -1, -2, 0;... % pt 2
%     0, 0, 0;...   % pt 3
%     -1, 2, 0;...  % pt 4
%     0, 0, -1;...  % pt 5
%     ]';

% define faces as a list of vertices numbered above
%   F = [...
%         1, 2, 3;...  % left wing
%         1, 3, 4;...  % right wing
%         1, 3, 5;...  % tail 
%         ];

% define colors for each face    
  myred = [1, 0, 0];
  mygreen = [0, 1, 0];
  myblue = [0, 0, 1];
  myyellow = [1, 1, 0];
  mycyan = [0, 1, 1];

%   facecolors = [...
%     mygreen;...    % left wing
%     mygreen;...    % right wing
%     myblue;...     % tail
%     ];
    facecolors = [...
        myred;...    
        myred;...    
        myred;...  
        myred;...
        myred;...
        myred;...
        myred;...
        myred;...
        myblue;...
        myblue;...
        myblue;...
        myblue;...
        myblue];
end
  