function dx = planar_quadrotor_dynamics(params, x, u, xref)
% x: [y; z; theta; ydot; zdot; thetadot]
% u: [F; M]
g = params(1);
m = params(2);
J = params(3);

f_x = [ x(4);
        x(5);
        x(6);
        0;
        -g;
        0 ];
g_x = [ 0               0;
        0               0;
        0               0;
        -sin(x(3))/m    0;
        cos(x(3))/m     0;
        0               1/J ];
        
% integrators for x, z
% f_x = [ f_x;
%         xref(1)-x(1);
%         xref(2)-x(2) ];
% g_x = [ g_x;
%         0               0;  
%         0               0 ];

dx = f_x + g_x*u;

end