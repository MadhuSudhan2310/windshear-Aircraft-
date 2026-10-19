%% windshear_params.m
% Parameters for the "Effect of wind shear on a PID-controlled aircraft" project.
% Run via build_windshear_pid_model (it evaluates this script in the base workspace).
%
% NOTE: The aircraft derivatives below are REPRESENTATIVE values for a
% medium transport in cruise (short-period wn ~ 2.7 rad/s, zeta ~ 0.46).
% Replace them with your own aircraft data (e.g. Nelson / Etkin / Stevens&Lewis)
% for your thesis. Everything else in the project updates automatically.

%% --- Flight condition -------------------------------------------------
V0   = 150;      % trim true airspeed [m/s]

%% --- Aircraft short-period + flight-path model --------------------------
%  alpha_dot = -Za*alpha_aero + q
%  q_dot     = k*(Mal*alpha_aero + Md*de) + Mq*q
%  theta_dot = q
%  h_dot     = Va*(theta - alpha)
%  alpha_aero = alpha - w_wind/Va        (downdraft w_wind > 0 reduces alpha)
%  k = (Va/V0)^2                         (dynamic-pressure scaling of moments)
Za  =  1.0;      % [1/s]   lift-curve term
Mal = -6.0;      % [1/s^2] static stability (Malpha)
Mq  = -1.5;      % [1/s]   pitch damping
Md  =  5.0;      % [1/s^2] elevator effectiveness (positive = nose-up command)

%% --- Airspeed response to shear -------------------------------------------
tauV = 15;       % [s] first-order airspeed recovery (phugoid / thrust lag proxy)
                 % dVa_dot = -dVa/tauV + d(u_wind)/dt

%% --- Actuator --------------------------------------------------------------
tau_act = 0.05;          % [s]   elevator servo lag
dmax    = deg2rad(25);   % [rad] elevator position limit

%% --- Wind shear (smooth headwind->tailwind + downdraft, microburst-like) --
ShearOn = 1;     % 1 = shear active, 0 = calm air
Ah      = 15;    % [m/s] headwind loss (+Ah headwind -> -Ah tailwind, i.e. 2*Ah swing)
Aw      = 8;     % [m/s] peak downdraft
t0      = 20;    % [s]   shear onset
Tshear  = 20;    % [s]   shear duration

%% --- PID gains --------------------------------------------------------------
% Inner loop: pitch attitude hold   (theta_cmd - theta) -> elevator
Kp = 2.0;  Ki = 0.8;  Kd = 0.8;  Nf = 20;
% Outer loop: altitude hold         (h_cmd - h) -> theta_cmd  (limited to +/-10 deg)
Kph = 0.006;  Kih = 0.0006;  Kdh = 0.015;  Nfh = 10;
theta_cmd_max = deg2rad(10);
hcmd  = 0;       % [m] altitude command (deviation from trim altitude)

%% --- Simulation -------------------------------------------------------------
tstop = 100;     % [s]
