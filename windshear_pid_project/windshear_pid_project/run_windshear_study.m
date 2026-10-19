function results = run_windshear_study()
%RUN_WINDSHEAR_STUDY  Parametric study: how wind shear degrades PID performance.
%   Study A: calm air vs. shear, pitch-hold only vs. pitch + altitude loop
%   Study B: shear-intensity sweep (Ah = 0..25 m/s)
%   Study C: inner-loop PID gain sets under shear (robustness / trade-off)
%   Produces figures + a results table (windshear_results.csv).

mdl = 'windshear_pid_aircraft';
if ~bdIsLoaded(mdl)
    if exist([mdl '.slx'],'file'), load_system(mdl); else, build_windshear_pid_model; end
end
evalin('base','windshear_params');
get = @(n) evalin('base',n);
put = @(n,v) assignin('base',n,v);

%% ---------- Study A : calm vs shear, with/without altitude loop ----------
cases = {'Calm, alt loop',      0, true;
         'Shear, pitch-hold only', 1, false;
         'Shear, alt loop ON',  1, true};
fig1 = figure('Name','Study A','Position',[100 100 900 700]);
kph0 = get('Kph'); kih0 = get('Kih'); kdh0 = get('Kdh');
for i = 1:size(cases,1)
    put('ShearOn',cases{i,2});
    if cases{i,3}, put('Kph',kph0); put('Kih',kih0); put('Kdh',kdh0);
    else,          put('Kph',0);    put('Kih',0);    put('Kdh',0); end
    o = sim(mdl,'StopTime','tstop');
    t = o.h_log.Time;
    subplot(4,1,1); hold on; plot(t,o.h_log.Data,'LineWidth',1.4);       ylabel('h dev [m]');
    subplot(4,1,2); hold on; plot(t,rad2deg(o.theta_log.Data),'LineWidth',1.4); ylabel('\theta [deg]');
    subplot(4,1,3); hold on; plot(t,rad2deg(o.de_log.Data),'LineWidth',1.4);    ylabel('\delta_e [deg]');
    subplot(4,1,4); hold on; plot(t,o.dVa_log.Data,'LineWidth',1.4);     ylabel('\DeltaV_a [m/s]'); xlabel('t [s]');
    R(i).name = cases{i,1}; %#ok<AGROW>
    R(i).h_dev = max(abs(o.h_log.Data));
    R(i).th_max = rad2deg(max(abs(o.theta_log.Data)));
    R(i).de_max = rad2deg(max(abs(o.de_log.Data)));
    R(i).h_end = o.h_log.Data(end);
end
subplot(4,1,1); legend(cases(:,1)','Location','best'); grid on;
for k=2:4, subplot(4,1,k); grid on; end
sgtitle('Study A: effect of wind shear and the altitude loop');
put('Kph',kph0); put('Kih',kih0); put('Kdh',kdh0); put('ShearOn',1);
results.A = struct2table(R);
disp(results.A);

%% ---------- Study B : intensity sweep ----------
Avals = 0:5:25;
hd = zeros(size(Avals)); hp = hd; thm = hd; dem = hd;
for i = 1:numel(Avals)
    put('Ah',Avals(i)); put('Aw',Avals(i)*8/15);
    % with altitude loop
    o = sim(mdl,'StopTime','tstop');
    hd(i) = max(abs(o.h_log.Data)); thm(i) = rad2deg(max(abs(o.theta_log.Data)));
    dem(i) = rad2deg(max(abs(o.de_log.Data)));
    % pitch-hold only
    put('Kph',0); put('Kih',0); put('Kdh',0);
    o = sim(mdl,'StopTime','tstop'); hp(i) = max(abs(o.h_log.Data));
    put('Kph',kph0); put('Kih',kih0); put('Kdh',kdh0);
end
put('Ah',15); put('Aw',8);
figure('Name','Study B','Position',[150 150 700 450]);
plot(Avals,hp,'o-',Avals,hd,'s-','LineWidth',1.5); grid on;
xlabel('Headwind loss A_h [m/s]'); ylabel('Peak altitude deviation [m]');
legend('Pitch-hold only','Pitch + altitude PID','Location','northwest');
title('Study B: altitude error vs shear intensity');
results.B = table(Avals(:),hp(:),hd(:),thm(:),dem(:), ...
    'VariableNames',{'Ah_mps','h_dev_pitchonly_m','h_dev_altloop_m','theta_max_deg','de_max_deg'});
disp(results.B);

%% ---------- Study C : inner-loop gain sets (pitch-hold mode, altitude loop off) ----------
gains = [1.0 0.3 0.4; 1.5 0.5 0.6; 2.0 0.8 0.8; 3.0 1.0 1.0];
Kp0 = get('Kp'); Ki0 = get('Ki'); Kd0 = get('Kd');
put('Kph',0); put('Kih',0); put('Kdh',0);
C = zeros(size(gains,1),4);
for i = 1:size(gains,1)
    put('Kp',gains(i,1)); put('Ki',gains(i,2)); put('Kd',gains(i,3));
    o = sim(mdl,'StopTime','tstop');
    C(i,:) = [max(abs(o.h_log.Data)), rad2deg(max(abs(o.theta_log.Data))), ...
              rad2deg(max(abs(o.de_log.Data))), ...
              sqrt(mean(rad2deg(o.theta_log.Data).^2))];
end
put('Kp',Kp0); put('Ki',Ki0); put('Kd',Kd0);
put('Kph',kph0); put('Kih',kih0); put('Kdh',kdh0);
results.C = table(gains(:,1),gains(:,2),gains(:,3),C(:,1),C(:,2),C(:,3),C(:,4), ...
    'VariableNames',{'Kp','Ki','Kd','h_dev_m','theta_max_deg','de_max_deg','theta_rms_deg'});
disp(results.C);

writetable(results.B,'windshear_results.csv');
saveas(fig1,'studyA_shear_response.png');
end
