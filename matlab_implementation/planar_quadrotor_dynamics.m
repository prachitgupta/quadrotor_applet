function dx = planar_quadrotor_dynamics(x, u)
% x: [y; z; theta; ydot; zdot; thetadot]
% u: [F; M]
global g m Ixx

f_x = [ x(4);
        x(5);
        x(6);
        0;
        -g;
        0   ];
g_x = [ 0               0;
        0               0;
        0               0;
        -sin(x(3))/m    0;
        cos(x(3))/m     0;
        0               1/Ixx   ];
dx = f_x + g_x*u;

end