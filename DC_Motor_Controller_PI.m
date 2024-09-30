clc;
close all;
clear all;

% motor parameters
R = 0.5;     % Resistance (ohms)
L = 0.01;    % Inductance (henries)
Km = 0.01;   % Motor constant (Nm/A)
Kb = 0.01;   % Back EMF constant (V/(rad/s))
J = 0.01;    % Rotor inertia (kg·m²)
B = 0.001;   % Viscous damping (N·m·s)

zeros2 = [];
poles2 = -R/L; % single pole at -R/L
gain2 = 1;
% P2 represents the transfer function from voltage to current (Electrical System)
P2 = zpk(zeros2, poles2, gain2);

zeros1 = [];
poles1 = roots([J*L, J*R + B*L, B*R, 0]); % Coefficients of s^3, s^2, s, and constant term
gain1 = Km;
% P1 is the transfer function from current to rotational speed. (Mechanical Dynamics)
P1 = zpk(zeros1, poles1, gain1);

% The plant model is P = P1*P2
% The complete plant model P is the combination of the electrical and mechanical dynamics.
P = P1*P2; 

% single loop (complete motor)
% Use a PID or PIDSTD object to define the desired controller structure
C = pidstd(1,1);
% Tune PI controller for target bandwidth is 0.2 rad/s
C = pidtune(P,C,0.2);

% cascading 
% PID tunning inner loop
C2 = pidtune(P2,pidstd(1,1),2);
% Inner loop system when the control loop is closed first
clsys = feedback(P2*C2,1); 
% PID tunning outer loop
% Plant seen by the outer loop controller C1 is clsys*P1
C1 = pidtune(clsys*P1,pidstd(1,1),0.2);

% single loop system for reference tracking 
sys1 = feedback(P*C,1);
sys1.Name = 'Single Loop';
% cascade system for reference tracking
sys2 = feedback(clsys*P1*C1,1); 
sys2.Name = 'Cascade';

% plot step response
figure;
step(sys1,'r',sys2,'b')
legend('show','location','southeast')
title('Figure-1: Reference Tracking')

% single loop system for rejecting d2
% P1: The mechanical part of the motor's dynamics (e.g., rotational inertia, damping).
% P2: The electrical part of the motor dynamics (e.g., inductance, resistance).
% C: The overall PID controller in single-loop configuration.
sysd1 = feedback(P1,P2*C); 
sysd1.Name = 'Single Loop';

% cascade system for rejecting d2
% Inner loop: This loop controls P2 (the electrical system) with the controller C2. 
% The output of this loop is P2*C2, which is the closed-loop transfer function of the inner loop.
% Outer loop: This loop controls P1 (the mechanical system) with the controller C1, 
% but the plant seen by this controller is actually the inner loop system.
sysd2 = P1/(1+P2*C2+P2*P1*C1*C2); 
sysd2.Name = 'Cascade';

% plot step response
figure;
step(sysd1,'r',sysd2,'b')
legend('show')
title('Figure-2: Disturbance Rejection')