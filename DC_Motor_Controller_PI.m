clc;
close all;
clear all;

R = 0.5;     % Resistance (ohms)
L = 0.01;    % Inductance (henries)
Km = 0.01;   % Motor constant (Nm/A)
Kb = 0.01;   % Back EMF constant (V/(rad/s))
J = 0.01;    % Rotor inertia (kg·m²)
B = 0.001;   % Viscous damping (N·m·s)
zeros2 = [];
poles2 = -R/L; % single pole at -R/L
gain2 = 1;
P2 = zpk(zeros2, poles2, gain2);
zeros1 = [];
poles1 = roots([J*L, J*R + B*L, B*R, 0]); % Coefficients of s^3, s^2, s, and constant term
gain1 = Km;
P1 = zpk(zeros1, poles1, gain1);

% The plant model is P = P1*P2
P = P1*P2; 
% Use a PID or PIDSTD object to define the desired controller structure
C = pidstd(1,1);
% Tune PI controller for target bandwidth is 0.2 rad/s
C = pidtune(P,C,0.2);

C2 = pidtune(P2,pidstd(1,1),2);

% Inner loop system when the control loop is closed first
clsys = feedback(P2*C2,1); 
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
sysd1 = feedback(P1,P2*C); 
sysd1.Name = 'Single Loop';
% cascade system for rejecting d2
sysd2 = P1/(1+P2*C2+P2*P1*C1*C2); 
sysd2.Name = 'Cascade';
% plot step response
figure;
step(sysd1,'r',sysd2,'b')
legend('show')
title('Figure-2: Disturbance Rejection')