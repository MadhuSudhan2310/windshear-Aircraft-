function build_windshear_pid_model()
%BUILD_WINDSHEAR_PID_MODEL  Programmatically builds the Simulink model
%   'windshear_pid_aircraft' : aircraft pitch/altitude dynamics + wind shear
%   + cascaded PID (altitude outer loop, pitch-attitude inner loop).
%
%   Usage:  >> build_windshear_pid_model
%   Requires: MATLAB + Simulink only (no Aerospace Blockset needed).

evalin('base','windshear_params');          % put parameters in base workspace
mdl = 'windshear_pid_aircraft';
if bdIsLoaded(mdl), close_system(mdl,0); end
new_system(mdl); open_system(mdl);

L = @(s) ['simulink/' s];
B = @(name,lib,x,y,varargin) addb(mdl,name,L(lib),x,y,varargin{:});

%% ---------------- Wind shear generator ----------------
B('Clock','Sources/Clock',30,40);
B('Bias_t0','Math Operations/Bias',100,40,'Bias','-t0');
B('Gain_1overT','Math Operations/Gain',170,40,'Gain','1/Tshear');
B('Sat_0_1','Discontinuities/Saturation',240,40,'UpperLimit','1','LowerLimit','0');
B('dUw_dt','User-Defined Functions/Fcn',320,20,'Expr','-Ah*pi/Tshear*sin(pi*u[1])');
B('Ww_downdraft','User-Defined Functions/Fcn',320,70,'Expr','Aw*sin(pi*u[1])^2');
B('ShearOn_duw','Math Operations/Gain',420,20,'Gain','ShearOn');
B('ShearOn_ww','Math Operations/Gain',420,70,'Gain','ShearOn');
ln(mdl,'Clock',1,'Bias_t0',1); ln(mdl,'Bias_t0',1,'Gain_1overT',1);
ln(mdl,'Gain_1overT',1,'Sat_0_1',1);
ln(mdl,'Sat_0_1',1,'dUw_dt',1);  ln(mdl,'Sat_0_1',1,'Ww_downdraft',1);
ln(mdl,'dUw_dt',1,'ShearOn_duw',1); ln(mdl,'Ww_downdraft',1,'ShearOn_ww',1);

%% ---------------- Airspeed deviation (dVa) ----------------
B('Gain_tauV','Math Operations/Gain',560,20,'Gain','-1/tauV');
B('Sum_dVa','Math Operations/Sum',630,20,'Inputs','++');
B('Int_dVa','Continuous/Integrator',700,20);
ln(mdl,'Int_dVa',1,'Gain_tauV',1); ln(mdl,'Gain_tauV',1,'Sum_dVa',1);
ln(mdl,'ShearOn_duw',1,'Sum_dVa',2); ln(mdl,'Sum_dVa',1,'Int_dVa',1);

%% ---------------- Aircraft dynamics ----------------
B('Mux_alpha_aero','Signal Routing/Mux',560,140,'Inputs','3');
B('Fcn_alpha_aero','User-Defined Functions/Fcn',640,140,'Expr','u[1]-u[2]/(V0+u[3])');
B('Fcn_k','User-Defined Functions/Fcn',640,200,'Expr','(1+u[1]/V0)^2');
B('Gain_Za','Math Operations/Gain',730,140,'Gain','-Za');
B('Sum_alpha_dot','Math Operations/Sum',800,140,'Inputs','++');
B('Int_alpha','Continuous/Integrator',870,140);
B('Gain_Mal','Math Operations/Gain',730,240,'Gain','Mal');
B('Gain_Md','Math Operations/Gain',730,290,'Gain','Md');
B('Sum_M','Math Operations/Sum',800,260,'Inputs','++');
B('Prod_k','Math Operations/Product',870,260);
B('Gain_Mq','Math Operations/Gain',870,330,'Gain','Mq');
B('Sum_q_dot','Math Operations/Sum',940,280,'Inputs','++');
B('Int_q','Continuous/Integrator',1010,280);
B('Int_theta','Continuous/Integrator',1080,280);
B('Mux_hdot','Signal Routing/Mux',1080,380,'Inputs','3');
B('Fcn_hdot','User-Defined Functions/Fcn',1150,380,'Expr','(V0+u[3])*(u[1]-u[2])');
B('Int_h','Continuous/Integrator',1230,380);

ln(mdl,'Int_alpha',1,'Mux_alpha_aero',1); ln(mdl,'ShearOn_ww',1,'Mux_alpha_aero',2);
ln(mdl,'Int_dVa',1,'Mux_alpha_aero',3);   ln(mdl,'Mux_alpha_aero',1,'Fcn_alpha_aero',1);
ln(mdl,'Int_dVa',1,'Fcn_k',1);
ln(mdl,'Fcn_alpha_aero',1,'Gain_Za',1); ln(mdl,'Gain_Za',1,'Sum_alpha_dot',1);
ln(mdl,'Int_q',1,'Sum_alpha_dot',2);   ln(mdl,'Sum_alpha_dot',1,'Int_alpha',1);
ln(mdl,'Fcn_alpha_aero',1,'Gain_Mal',1); ln(mdl,'Gain_Mal',1,'Sum_M',1);
ln(mdl,'Gain_Md',1,'Sum_M',2);
ln(mdl,'Sum_M',1,'Prod_k',1); ln(mdl,'Fcn_k',1,'Prod_k',2);
ln(mdl,'Prod_k',1,'Sum_q_dot',1);
ln(mdl,'Int_q',1,'Gain_Mq',1); ln(mdl,'Gain_Mq',1,'Sum_q_dot',2);
ln(mdl,'Sum_q_dot',1,'Int_q',1); ln(mdl,'Int_q',1,'Int_theta',1);
ln(mdl,'Int_theta',1,'Mux_hdot',1); ln(mdl,'Int_alpha',1,'Mux_hdot',2);
ln(mdl,'Int_dVa',1,'Mux_hdot',3);   ln(mdl,'Mux_hdot',1,'Fcn_hdot',1);
ln(mdl,'Fcn_hdot',1,'Int_h',1);

%% ---------------- Elevator actuator ----------------
B('Elevator_actuator','Continuous/Transfer Fcn',560,300,'Numerator','1','Denominator','[tau_act 1]');
ln(mdl,'Elevator_actuator',1,'Gain_Md',1);

%% ---------------- Cascaded PID ----------------
B('h_cmd','Sources/Constant',30,300,'Value','hcmd');
B('Sum_h_err','Math Operations/Sum',100,300,'Inputs','+-');
B('PID_altitude','Continuous/PID Controller',170,290, ...
  'P','Kph','I','Kih','D','Kdh','N','Nfh', ...
  'LimitOutput','on','UpperSaturationLimit','theta_cmd_max', ...
  'LowerSaturationLimit','-theta_cmd_max','AntiWindupMode','clamping');
B('Sum_theta_err','Math Operations/Sum',270,300,'Inputs','+-');
B('PID_pitch','Continuous/PID Controller',340,290, ...
  'P','Kp','I','Ki','D','Kd','N','Nf', ...
  'LimitOutput','on','UpperSaturationLimit','dmax', ...
  'LowerSaturationLimit','-dmax','AntiWindupMode','clamping');
ln(mdl,'h_cmd',1,'Sum_h_err',1);   ln(mdl,'Int_h',1,'Sum_h_err',2);
ln(mdl,'Sum_h_err',1,'PID_altitude',1);
ln(mdl,'PID_altitude',1,'Sum_theta_err',1); ln(mdl,'Int_theta',1,'Sum_theta_err',2);
ln(mdl,'Sum_theta_err',1,'PID_pitch',1);   ln(mdl,'PID_pitch',1,'Elevator_actuator',1);

%% ---------------- Logging & scope ----------------
logs = {'h_log','Int_h'; 'theta_log','Int_theta'; 'alpha_log','Int_alpha'; ...
        'dVa_log','Int_dVa'; 'de_log','Elevator_actuator'; 'ww_log','ShearOn_ww'};
for i = 1:size(logs,1)
    nm = ['Log_' logs{i,1}];
    B(nm,'Sinks/To Workspace',1320,40+45*i,'VariableName',logs{i,1},'SaveFormat','Timeseries');
    ln(mdl,logs{i,2},1,nm,1);
end
B('Scope','Sinks/Scope',1320,360,'NumInputPorts','4');
ln(mdl,'Int_h',1,'Scope',1); ln(mdl,'Int_theta',1,'Scope',2);
ln(mdl,'Elevator_actuator',1,'Scope',3); ln(mdl,'ShearOn_ww',1,'Scope',4);

set_param(mdl,'StopTime','tstop','Solver','ode45','MaxStep','0.02');
save_system(mdl);
fprintf('Built and saved model "%s". Press Run, or call run_windshear_study.\n',mdl);
end

%% ----- helpers -----
function p = addb(mdl,name,src,x,y,varargin)
p = [mdl '/' name];
add_block(src,p,'Position',[x y x+60 y+30]);
for k = 1:2:numel(varargin), set_param(p,varargin{k},varargin{k+1}); end
end

function ln(mdl,a,pa,b,pb)
add_line(mdl,[a '/' num2str(pa)],[b '/' num2str(pb)],'autorouting','on');
end
