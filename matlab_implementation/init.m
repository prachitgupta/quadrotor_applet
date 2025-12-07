%% model params
g = 9.81;
m = .5;
J = .1;
params = [g, m, J];
nx = 6;
nu = 2;
nr = 6;

%% known (wrong) params
g_est = g;
%m_est = 0.5*m;
%J_est = .2*J;
%m_est = 1.3*m;
%J_est = 2.*J;
% g_est = g;
m_est = m;
J_est = J;
params_est = [g_est, m_est, J_est];

%% sim params
T = 5000;
ts = 0.01;
tvec = (0:ts:T)';
X0 = [0, 0, 0, 0, 0, 0]';    % [x; z; theta; ydot; zdot; thetadot; (x_int; z_int)]
[K_lqr, ustar, Am, Bm] = get_LQR(params_est);  % desired system Am, Bm:
                                               % from linearized model (around hover) with LQR control
% [Am, Bm] = second_order_reference_model();    % avoid this way, need to
%                                               % do inner-outer loop

%% reference trajectory
traj_shape = 'circle';
% traj_shape = 'figure 8';
traj_shape = [1, 1, 0, 0, 0, 0]';
% traj_shape = 'persistently excited';
radius = 1.;
omega = 1.;

%% controllers to use: true/false
controller_delay = 0;   % u is delayed by N time steps
% enable_gravity_compensation = true;
enable_LQR = true;
enable_MRAC = false;
enable_constraints = false;

%% MRAC projection operator
enable_MRAC_proj = true;
Kx_max = [80 80];
Kr_max = [80 80];
tolerance = [.01 .01];

%% MRAC gains
Gamma_x = eye(nx) * 5000;  % learning gain - Kx
Gamma_r = eye(nr) * 5000;  % learning gain - Kr

%% matched uncertainty
enable_matched_uncertainty = false;

%% measurement noise (for parameters drift)
enable_measurement_noise = true;
%% constraints on state and control
xvec_min =  [-30; 0; -pi/2; -6; -6; -8];
xvec_max =  [30; 20; pi/2; -6; -6; -8];
uvec_min = [0;-18];
uvec_max = [40;18];

clamp = @(V,L,U)min(max(V,L), U);
%% functions
function [K, ustar, Am, Bm] = get_LQR(params)
    g = params(1);
    m = params(2);
    J = params(3);
    A = [ 0 0 0 1 0 0;
          0 0 0 0 1 0;
          0 0 0 0 0 1;
          0 0 -g 0 0 0;
          0 0 0 0 0 0;
          0 0 0 0 0 0 ];
    B = [ 0     0;
          0     0;
          0     0;
          0     0;
          1/m   0;
          0     1/J ];
    % Augment integrators for x, z
    % A = [ A, zeros(6,2);
    %       -1 0 0 0 0 0 0 0;
    %       0 -1 0 0 0 0 0 0 ];
    % B = [ B; zeros(2,2) ];
    K = lqr(A, B, diag([10, 10, 1, 5, 5, 1]), eye(2));
    ustar = [m*g; 0];
    % closed-loop:  xdot = Am*x + Bm*xref
    % open_loop:    xdot = A*x + B*u + E*r (last term introduced by integrators)
    % E = [ zeros(6,8);
    %       1 0 0 0 0 0 0 0;
    %       0 1 0 0 0 0 0 0 ];
    Am = A - B*K;
    % Bm = B*K + E;
    Bm = B*K;
end

function [A, B] = second_order_reference_model()
    % reference: [x; z]
    x_damp = .9;
    z_damp = .9;
    t_damp = .7;
    x_freq = 2.;
    z_freq = 2.;
    t_freq = 3.;
    % inner PD loops: theta_ref = kp*(x - xref) + kd*(xd - xdref)
    kp = 2.; kd = 3.;
    A = [ 0, 0, 0, 1, 0, 0;
          0, 0, 0, 0, 1, 0;
          0, 0, 0, 0, 0, 1;
          -x_freq^2,    0,          0,          -2*x_damp*x_freq,   0,                  0;
          0,            -z_freq^2,  0,          0,                  -2*z_damp*z_freq,   0;
          -t_freq^2*kp, 0,          -t_freq^2,  -t_freq^2*kd,       0,                  -2*t_damp*t_freq ];
    B = [ 0             0;
          0             0;
          0             0;
          x_freq^2      0;
          0             z_freq^2;
          t_freq^2*kp   0 ];
end
