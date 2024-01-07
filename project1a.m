% Mikhail Smirnov EENG720: Modern Control Theory
% Design Project #1: PD Controller

% Simulate closed loop: Gcl(s) = Gc(s) Gp(s) / (1 + Gc(s) Gp(s))
% Gp(s) = 1 / (s^2 + s)
Gp = tf(1, [1 1 0]); % Plant transfer function in s-domain

% Uncompensated closed-loop transfer function in s-domain
Gcl_uncomp = feedback(Gp,1)

% Part 1: Proportional and derivative gain values
Kp1 = 25;
Kd1 = 6;

% PD Controller transfer function in s-domain ( = Kp + Kd s)
Gc = tf([Kd1 Kp1], 1);
% Loop gain: L(s) = Gc(s) * Gp(s)
L = Gc * Gp;
% Closed loop transfer function with PD in s-domain
Gcl = feedback(L,1)

% Discretized
omega_s = 100; % Sampling frequency
T = (2 * pi)/omega_s % Sampling period
% G(s) ---ZOH---> G(z)
Gz_zoh = c2d(Gp, T, 'zoh') % Discretized plant
% C(s) ---tustin---> C(z)
Cz_tustin = c2d(Gc, T, 'tustin') % Discretized controller
Lz = Gz_zoh * Cz_tustin; % Discretized loop gain

% Discretized Closed Loop Transfer Function of Uncompensated
Gcl_uncomp_z = feedback(Gz_zoh, 1)
% Discretized Closed Loop Transfer Function with PD
Gcl_z = feedback(Lz, 1)
% PLOTTING ---------------------------------------------------
% Simulate Gcl(s)
figure(1)
subplot(1,2,1)
hold on
step(Gcl_uncomp) % Step response of uncompensated
step(Gcl) % Step response of Part 1 PD
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
