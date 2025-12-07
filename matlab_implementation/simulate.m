clear
clc
init

%% MRAC
Lambda = [1/m 0; 0 1/J];
Lambda_est = [1/m_est 0; 0 1/J_est];
A = [ 0 0 0 1 0 0;
      0 0 0 0 1 0;
      0 0 0 0 0 1;
      0 0 0 0 0 0;
      0 0 0 0 0 0;
      0 0 0 0 0 0 ];
G = [0; 0; 0; 0; -g; 0];
% augment intergrators
% A = [ A, zeros(6,2);
%       -1 0 0 0 0 0 0 0;
%       0 -1 0 0 0 0 0 0 ];
% G = [ G; zeros(2,1) ];

P = lyap(Am', eye(nx));
Kx = zeros(nx, nu);
Kr = zeros(nr, nu);
% Kx = ones(nx, nu) * 1;
% Kr = ones(nr, nu) * 1;

%% sim
xvec = zeros(length(tvec), nx);     % actual
rvec = zeros(length(tvec), nr);     % reference
xmvec = zeros(length(tvec), nx);    % nominal (xmdot = Am*xm + Bm*r)
uvec = zeros(length(tvec), nu);
evec = zeros(length(tvec), nx);      % MRAC: e=x-xm
normEvec = zeros(length(tvec), 1);
normKxvec = zeros(length(tvec), 1);
normKrvec = zeros(length(tvec), 1);
normKxsvec = zeros(length(tvec), 1); % Kx_star
normKrsvec = zeros(length(tvec), 1); % Kr_star

% 3D arrays
% get value at time t:  Kt = squeeze(Kxvec(t,:,:))
% assignment:           Kxvec(t,:,:) = Kt
% Kxvec = zeros(length(tvec), nx, nu);
% Krvec = zeros(length(tvec), nr, nu);
% Kxvec = zeros(length(tvec), nx);

x0 = X0;
x0m = X0;

for i = 1:length(tvec)
    r = generate_trajectory(traj_shape, radius, omega, tvec(i));
    % r = generate_trajectory(traj_shape, 1.+sin(.5*tvec(i)), 2., tvec(i));
    rvec(i,:) = r';
    xvec(i,:) = x0';
    xmvec(i,:) = x0m';
    
    x0_pristine = x0;   % not corrupted by measurement noise
    if enable_measurement_noise
        x0 = add_measurement_noise(x0, tvec(i));
    end
    e = x0 - x0m;
    u = zeros(nu, 1);
    evec(i,:) = e';

    % ----------------- MRAC dynamics ---------------- %
    B = [ zeros(3, nu);
          -sin(x0(3))   0;
          cos(x0(3))    0;
          0             1 ];
    if enable_MRAC_proj
        Kx = Kx + Gamma_x*proj(Kx, -x0*e'*P*B, Kx_max, tolerance)*ts;
        Kr = Kr + Gamma_r*proj(Kr, -r*e'*P*B, Kr_max, tolerance)*ts;
    else
        Kx = Kx - (Gamma_x * x0 * e' * P * B)*ts;
        Kr = Kr - (Gamma_r * r * e' * P * B)*ts;
    end
    % Kxvec(i,:,:) = Kx;
    % Krvec(i,:,:) = Kr;
    % Kxvec(i,:) = Kx(:,1)';

    % Actual params Kx_star, Ku_star from true values of m,J
    B_temp = B*Lambda;
    Kx_star = (Am - A)' / B_temp';
    Kr_star = Bm' / B_temp';
    % Kx = Kx_star;
    % Kr = Kr_star;
    
    % ----------------- Latest control --------------- %
    % u = planar_quadrotor_feedback_linearization_control(params, x0-r);
    if enable_LQR
        u = u + K_lqr*(r - x0) + ustar;
    end
    % if enable_gravity_compensation
    %     u = u + [m_est*g_est/cos(x0(3)); 0];
    % end
    if enable_MRAC
        u = u + Kx'*x0 + Kr'*r;
        % u = u + Kx'*x0;
        % u = u + Kx'*x0 + Kr'*(rs - pinv(B*Kr')*G);
        % u = u + Kx'*x0 + Kr'*r - pinv(B*Lambda_est)*G;
    end
    uvec(i,:) = u';

    % ----------------- Applied control (w/ time delay) --------------- %
    if i > controller_delay
        u = uvec(i-controller_delay, :)';
    else
        u = zeros(nu, 1);
    end

    % ----------------- Matched uncertainty --------------- %
    if enable_matched_uncertainty
        % u(1) = u(1) + 20.;
        % u(2) = u(2) - .1;
        % u(1) = u(1) + 2.*sin(2.*tvec(i));
        u = u + [3.*sin(pi*tvec(i)); .2*sin(2*pi*tvec(i))];
    end
    
    % ------------------------------------------------- %
    % B_temp = [ zeros(3, nu);
    %            -sin(x0(3))/m   0;
    %            cos(x0(3))/m-g/u(1)    0;
    %            0             1/J ];
    % Kx_star = (Am - A)' / B_temp';
    % Kr_star = Bm' / B_temp';

    normEvec(i) = norm(e, inf);
    normKxvec(i) = norm(Kx, inf);
    normKrvec(i) = norm(Kr, inf);
    normKxsvec(i) = norm(Kx_star, inf);
    normKrsvec(i) = norm(Kr_star, inf);

    dx = planar_quadrotor_dynamics(params, x0_pristine, u, r);
    x0 = x0_pristine + dx*ts;
    x0m = x0m + (Am*x0m + Bm*r)*ts;
    % x0m = x0m + (Am*x0m + Bm*r + G)*ts;
end

%%%constraints
if enable_constraints
    xvec = clamp(xvec,xvec_min',xvec_max');
    uvec = clamp(uvec,uvec_min', uvec_max');
end

%% plot
close all

figure(1); grid on; hold on;
% step = ceil(length(tvec)/600);
step = 10;
plot(rvec(:,1), rvec(:,2), 'r-o', 'LineWidth', 1.5)
plot(xmvec(1:step:end,1), xmvec(1:step:end,2), 'k', 'LineWidth', 1.)
plot(xvec(1:step:end,1), xvec(1:step:end,2), 'b-*', 'LineWidth', 1.)
legend('$(x_{des},z_{des})$', '$(x_m,z_m)$', '$(x,z)$', 'Interpreter', 'latex', 'FontSize', 12)
xlabel('$x~(m)$', 'Interpreter', 'latex', 'FontSize', 14)
ylabel('$z~(m)$', 'Interpreter', 'latex', 'FontSize', 14)
axis equal
title('Planar quadrotor trajectory')



figure(2)

subplot(421); grid on; hold on;
plot(tvec, rvec(:,1), 'k--', 'LineWidth', 1.)
plot(tvec, xmvec(:,1), 'r-', 'LineWidth', 1.5)
plot(tvec, xvec(:,1), 'k-', 'LineWidth', 1.5)
legend('$x_{des}$', '$x_m$', '$x$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$x~(m)$', 'Interpreter', 'latex', 'FontSize', 12)
title('Pose')

subplot(422); grid on; hold on;
plot(tvec, rvec(:,4), 'k--', 'LineWidth', 1.)
plot(tvec, xmvec(:,4), 'r-', 'LineWidth', 1.5)
plot(tvec, xvec(:,4), 'k-', 'LineWidth', 1.5)
legend('$\dot{x}_{des}$', '$\dot{x}_m$', '$\dot{x}$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$\dot{x}~(m/s)$', 'Interpreter', 'latex', 'FontSize', 12)
title('Pose derivative')

subplot(423); grid on; hold on;
plot(tvec, rvec(:,2), 'k--', 'LineWidth', 1.)
plot(tvec, xmvec(:,2), 'r-', 'LineWidth', 1.5)
plot(tvec, xvec(:,2), 'k-', 'LineWidth', 1.5)
legend('$z_{des}$', '$z_m$', '$z$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$z~(m)$', 'Interpreter', 'latex', 'FontSize', 12)

subplot(424); grid on; hold on;
plot(tvec, rvec(:,5), 'k--', 'LineWidth', 1.)
plot(tvec, xmvec(:,5), 'r-', 'LineWidth', 1.5)
plot(tvec, xvec(:,5), 'k-', 'LineWidth', 1.5)
legend('$\dot{z}_{des}$', '$\dot{z}_m$', '$\dot{z}$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$\dot{z}~(m/s)$', 'Interpreter', 'latex', 'FontSize', 12)

subplot(425); grid on; hold on;
plot(tvec, rvec(:,3)*180/pi, 'k--', 'LineWidth', 1.)
plot(tvec, xmvec(:,3)*180/pi, 'r-', 'LineWidth', 1.5)
plot(tvec, xvec(:,3)*180/pi, 'k-', 'LineWidth', 1.5)
legend('$\theta_{des}$', '$\theta_m$', '$\theta$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$\theta~(^\circ)$', 'Interpreter', 'latex', 'FontSize', 12)

subplot(426); grid on; hold on;
plot(tvec, rvec(:,6)*180/pi, 'k--', 'LineWidth', 1.)
plot(tvec, xmvec(:,6)*180/pi, 'r-', 'LineWidth', 1.5)
plot(tvec, xvec(:,6)*180/pi, 'k-', 'LineWidth', 1.5)
legend('$\dot{\theta}_{des}$', '$\dot{\theta}_m$', '$\dot{\theta}$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$\dot{\theta}~(^\circ/s)$', 'Interpreter', 'latex', 'FontSize', 12)

subplot(427); grid on; hold on;
plot(tvec, uvec(:,1), 'k-', 'LineWidth', 1.5)
ylabel('$F~(N)$', 'Interpreter', 'latex', 'FontSize', 12)
xlabel('Time (s)')
title('Body thrust')

subplot(428); grid on; hold on;
plot(tvec, uvec(:,2), 'k-', 'LineWidth', 1.5)
ylabel('$M~(Nm)$', 'Interpreter', 'latex', 'FontSize', 12)
xlabel('Time (s)')
title('Body torque')



figure(3)

subplot(311); grid on; hold on;
% plot(tvec, evec)
plot(tvec, normEvec, 'k-', 'LineWidth', 1.5)
% set(gca, 'YScale', 'log')
ylabel('$\|\mathbf{e}(t)\|_{\infty}$', 'Interpreter', 'latex', 'FontSize', 12)

subplot(312); grid on; hold on;
plot(tvec, normKxsvec, 'k--', 'LineWidth', 1.)
plot(tvec, normKxvec, 'k-', 'LineWidth', 1.5)
% plot(tvec, Kxvec, 'LineWidth', .5)
legend('$\|\mathbf{K}_x^*\|$', '$\|\mathbf{K}_x\|$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$\|\mathbf{K}_x(t)\|_{\infty}$', 'Interpreter', 'latex', 'FontSize', 12)

subplot(313); grid on; hold on;
plot(tvec, normKrsvec, 'k--', 'LineWidth', 1.)
plot(tvec, normKrvec, 'k-', 'LineWidth', 1.5)
legend('$\|\mathbf{K}_r^*\|$', '$\|\mathbf{K}_r\|$', 'Interpreter', 'latex', 'FontSize', 12)
ylabel('$\|\mathbf{K}_r(t)\|_{\infty}$', 'Interpreter', 'latex', 'FontSize', 12)
xlabel('Time (s)')

% figure(4)

% subplot(311); grid on; hold on;
% % plot(tvec, evec)
% plot(tvec, normEvec, 'k-', 'LineWidth', 1.5)
% % set(gca, 'YScale', 'log')
% ylabel('$\|\mathbf{e}(t)\|_{\infty}$', 'Interpreter', 'latex', 'FontSize', 12)

% subplot(312); grid on; hold on;
% plot(tvec, Kx_star, 'k--', 'LineWidth', 1.)
% plot(tvec, Kx, 'k-', 'LineWidth', 1.5)
% % plot(tvec, Kxvec, 'LineWidth', .5)
% legend('$\|\mathbf{K}_x^*\|$', '$\|\mathbf{K}_x\|$', 'Interpreter', 'latex', 'FontSize', 12)
% ylabel('$\|\mathbf{K}_x(t)\|_{\infty}$', 'Interpreter', 'latex', 'FontSize', 12)

% subplot(313); grid on; hold on;
% plot(tvec, Kr_star, 'k--', 'LineWidth', 1.)
% plot(tvec, Kr, 'k-', 'LineWidth', 1.5)
% legend('$\|\mathbf{K}_r^*\|$', '$\|\mathbf{K}_r\|$', 'Interpreter', 'latex', 'FontSize', 12)
% ylabel('$\|\mathbf{K}_r(t)\|_{\infty}$', 'Interpreter', 'latex', 'FontSize', 12)
% xlabel('Time (s)')



%%
function r = generate_trajectory(type, radius, omega, t)
    if strcmp(type, 'circle')
        r = [ radius * sin(omega*t);
              radius * cos(omega*t);
              0;
              radius * omega * cos(omega*t);
              -radius * omega * sin(omega*t);
              0 ];
    elseif strcmp(type, 'figure 8')
        r = [ radius * sin(omega*t);
              radius * cos(.5*omega*t);
              0;
              radius * omega * cos(omega*t);
              -.5*radius * omega * sin(.5*omega*t);
              0 ];
    elseif strcmp(type, 'persistently excited')
        d = .1;
        a = pi/36;
        r = [ d * sin(2*t);
              d * cos(5*t);
              a * sin(7*t);
              2*d * cos(2*t);
              -5*d * sin(5*t);
              7*a * cos(7*t) ];
    else
        % r = [1 2 0 0 0 0]';
        r = type;
    end
    % if strcmp(type, 'circle')
    %     r = [ radius * sin(omega*t);
    %           radius * cos(omega*t) ];
    % elseif strcmp(type, 'figure 8')
    %     r = [ radius * sin(omega*t);
    %           radius * cos(.5*omega*t) ];
    % elseif strcmp(type, 'persistently excited')
    %     d = .1;
    %     r = [ d * sin(2*t);
    %           d * cos(5*t) ];
    % else
    %     r = type;
    % end
    % % theta reference
    % kp = 1.;
    % kd = 2.;
    % ax = kp*(x(1) - r(1)) + kd*(x(4) - 0);
    % % r(3) = kp*(x(1) - r(1)) + kd*(x(4) - 0);
    % r(3) = asin(ax/9.81);
end

function y = add_measurement_noise(x, t)
    d = .1;
    a = pi/36;
    y = x + [ d*sin(20*t);
              d*cos(50*t);
              0;
              0;
              0;
              0 ];
    % y = x + [-.05; -.1; 0; .1; .1; -.05];
end
