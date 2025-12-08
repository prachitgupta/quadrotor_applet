clear
clc

%% model params
global g m Ixx
g = 9.81;
m = .5;
Ixx = .1;

%% sim params
T = 10;
ts = .01;
tvec = (0:ts:T)';
X0 = [3, 0, -pi/8, 0, 0, 0]';    % [y; z; theta; ydot; zdot; thetadot]
xref = [0, 0, 0, 0, 0, 0]';

%% sim
xvec = zeros(length(tvec), 6);
uvec = zeros(length(tvec), 2);
x0 = X0;
normvec = zeros(length(tvec), 1);
cvec = zeros(length(tvec), 1);  % level set for the position subsystem - c = (618239*F^2)/(462818*m^2)
Vvec = zeros(length(tvec), 1);  % lyapunov value - V(z)<=c
Vdotvec = zeros(length(tvec), 1);

% position subsystem - lyapunov function V(z)=z^T P z
P = [ 809/260       181/52      413/260     1/20;
      181/52        691/104     187/52      23/104;
      413/260       187/52      67/20       71/260;
      1/20          23/104      71/260      67/520 ];

for i = 1:length(tvec)
    xvec(i,:) = x0';
    normvec(i) = norm(x0);
    u = planar_quadrotor_feedback_linearization_control(x0-xref);
    % u = max(-umax, min(umax, u));   % saturation

    % -------- Position subsystem - level set -------- %
    F = u(1);
    z = [ x0(1);
          x0(4);
          -F*sin(x0(3))/m;
          -F*cos(x0(3))*x0(6)/m ];
    Vvec(i) = z'*P*z;
    cvec(i) = (618239*F^2)/(462818*m^2);
    Vdotvec(i) = -norm(z)^2;    % LfV = - z1^2 - z2^2 - z3^2 - z4^2 (from lyapunov.m)
    % -------------------------------------------------%

    dx = planar_quadrotor_dynamics(x0, u);
    x0 = x0 + dx*ts;
    uvec(i,:) = u';
end

%% plot
close all

figure(1); grid on; hold on;
plot(xvec(1:10:end,1), xvec(1:10:end,2), 'k*', 'LineWidth', 2)
xlabel('$x~(m)$', 'Interpreter', 'latex', 'FontSize', 14)
ylabel('$z~(m)$', 'Interpreter', 'latex', 'FontSize', 14)
axis equal
title('Planar quadrotor trajectory')





figure(2)

subplot(211); grid on; hold on;
plot(tvec, normvec, 'b-', 'LineWidth', 1.5)
yline(norm(X0), 'r--', 'LineWidth', 2)
legend('$\|x(t)\|$', '$\|x(0)\|$', 'Interpreter', 'latex', 'FontSize', 12)
xlabel('Time (s)'); ylabel('$\|x\|$', 'Interpreter', 'latex', 'FontSize', 14);
title('Norm of the state')

subplot(212); grid on; hold on;
plot(tvec, cvec, 'r--', 'LineWidth', 2)
plot(tvec, Vvec, 'b-', 'LineWidth', 1.5)
plot(tvec, Vdotvec, 'b--', 'LineWidth', 1)
legend('$c_F$', '$V(x)$', '$\dot{V}(x)$', 'Interpreter', 'latex', 'FontSize', 12)
xlabel('Time (s)'); ylabel('$V(x)$', 'Interpreter', 'latex', 'FontSize', 14);
title('Region of attraction V(x) \leq c_F for 2^{nd} (position) subsystem')


% figure(4); grid on; hold on;
% plot(tvec, cvec, 'r--', 'LineWidth', 2)
% plot(tvec, Vvec, 'b-', 'LineWidth', 1.5)
% plot(tvec, Vdotvec, 'b--', 'LineWidth', 1)
% legend('$c_F$', '$V(x)$', '$\dot{V}(x)$', 'Interpreter', 'latex', 'FontSize', 12)
% xlabel('Time (s)'); ylabel('$V(x)$', 'Interpreter', 'latex', 'FontSize', 14);
% title('Evolution of $V(x)$, $x(0)\in\Omega_{cx}$', 'Interpreter', 'latex', 'FontSize', 14)




figure(3)

subplot(421); grid on; hold on;
plot(tvec, xvec(:,1), 'k-', 'LineWidth', 1)
ylabel('$x~(m)$', 'Interpreter', 'latex', 'FontSize', 12)
title('Pose')

subplot(422); grid on; hold on;
plot(tvec, xvec(:,4), 'k-', 'LineWidth', 1)
ylabel('$\dot{x}~(m/s)$', 'Interpreter', 'latex', 'FontSize', 12)
title('Pose derivative')

subplot(423); grid on; hold on;
plot(tvec, xvec(:,2), 'k-', 'LineWidth', 1)
ylabel('$z~(m)$', 'Interpreter', 'latex', 'FontSize', 12)

subplot(424); grid on; hold on;
plot(tvec, xvec(:,5), 'k-', 'LineWidth', 1)
ylabel('$\dot{z}~(m/s)$', 'Interpreter', 'latex', 'FontSize', 12)

subplot(425); grid on; hold on;
plot(tvec, xvec(:,3)*180/pi, 'k-', 'LineWidth', 1)
ylabel('$\theta~(^\circ)$', 'Interpreter', 'latex', 'FontSize', 12)

subplot(426); grid on; hold on;
plot(tvec, xvec(:,6)*180/pi, 'k-', 'LineWidth', 1)
ylabel('$\dot{\theta}~(^\circ/s)$', 'Interpreter', 'latex', 'FontSize', 12)

subplot(427); grid on; hold on;
plot(tvec, uvec(:,1), 'k-', 'LineWidth', 1)
ylabel('$F~(N)$', 'Interpreter', 'latex', 'FontSize', 12)
xlabel('Time (s)')
title('Body thrust')

subplot(428); grid on; hold on;
plot(tvec, uvec(:,2), 'k-', 'LineWidth', 1)
ylabel('$M~(Nm)$', 'Interpreter', 'latex', 'FontSize', 12)
xlabel('Time (s)')
title('Body torque')
