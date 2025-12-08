function u = planar_quadrotor_feedback_linearization_control(x)
% x: [y; z; theta; ydot; zdot; thetadot]

global g m Ixx

%% altitude (z) subsystem (dim=2) - control body thrust F
% diffeomorphic transformation z=T(x) on R^2
z = [ x(2);
      x(5) ];

% control of feedback linearized system - 2nd-order integrator
A = [ 0 1;
      0 0 ];
B = [ 0;
      1 ];
K = place(A, B, [-1+j, -1-j]);
v = -K*z;

% control of original system
F = m*(v + g)/cos(x(3));

%% position (y) subsystem (dim=4) - control body torque M
% diffeomorphic transformation z=T(x) on |theta|<pi/2 and F!=0
z = [ x(1);
      x(4);
      -F*sin(x(3))/m;
      -F*cos(x(3))*x(6)/m ];

% control of feedback linearized system - 4th-order integrator
A = [ 0 1 0 0;
      0 0 1 0;
      0 0 0 1;
      0 0 0 0 ];
B = [ 0;
      0;
      0;
      1 ];
K = place(A, B, [-1+j, -1-j, -2+j, -2-j]);
v = -K*z;

% control of original system
Lf4_y = F*sin(x(3))*x(6)^2/m;
LgLf3_y = -F*cos(x(3))/(m*Ixx);
M = (v - Lf4_y)/LgLf3_y;

%% Total control
u = [F; M];

end