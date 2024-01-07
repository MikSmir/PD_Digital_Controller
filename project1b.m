% Mikhail Smirnov EENG720: Modern Control Theory
% Design Project #1: PD Controller

% Simulate closed loop: Gcl(s) = Gc(s) Gp(s) / (1 + Gc(s) Gp(s))
% Gp(s) = 1 / (s^2 + s)
Gp = tf(1, [1 1 0]); % Plant transfer function in s-domain

% Uncompensated closed-loop transfer function in s-domain:
Gcl_uncomp = feedback(Gp,1)

% Part 2: Proportional and derivative gain values
Kp2 = 100;
Kd2 = 15;

% PD Controller transfer function in s-domain ( = Kp + Kd s)
Gc = tf([Kd2 Kp2], 1);
% Loop gain: L(s) = Gc(s) * Gp(s)
L = Gc * Gp;
% Closed loop transfer function with PD in s-domain
Gcl = feedback(L,1)

% Discretized
% G(s) ---ZOH---> G(z)
omega_s = 200; % Sampling frequency
T = (2 * pi)/omega_s % Sampling period
Gz_zoh = c2d(Gp, T, 'zoh') % Discretized plant
% C(s) ---tustin---> C(z)
Cz_tustin = c2d(Gc, T, 'tustin') % Discretized controller
Lz = Gz_zoh * Cz_tustin % Discretized loop gain

% Discretized Closed Loop Transfer Function of Uncompensated
Gcl_uncomp_z = feedback(Gz_zoh, 1)
% Discretized Closed Loop Transfer Function with PD
Gcl_z = feedback(Lz, 1)

% PLOTTING ----------------------------------------------------
% Simulate Gcl(s)
figure(1)
subplot(1,2,1)
hold on
step(Gcl_uncomp)
step(Gcl)
title("Analog Step Response $G_{cl}(s)$", 'Interpreter', 'latex')
legend({"Uncompensated", "PD Compensated"}, 'Interpreter' ,'latex', ...
    'Location', 'southeast')
hold off

% Simulate Gcl(z)
subplot(1,2,2)
hold on
step(Gcl_uncomp_z)
step(Gcl_z)
title("Discretized Step Response of $G_{cl}(z)$", 'Interpreter','latex')
legend({"Uncompensated", "PD Compensated"}, 'Interpreter' ,'latex', ...
    'Location', 'southeast')
hold off
xlim([0 12])
ylim([0 1.2])