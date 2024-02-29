clear
clc

load('C:\Users\deshm\OneDrive\Documents\GitHub\VehicleModel\EVsim\Panasonic-18650PF-Data-master\Panasonic 18650PF Data\25degC\5 pulse disch\03-11-17_08.47 25degC_5Pulse_HPPC_Pan18650PF.mat')

%%
time = meas.Time;
voltage = meas.Voltage;
current = meas.Current;

inputData = [time, current];

% Initial parameters for the model
r0 = 0.02; % ohm
r1 = 0.01; % ohm
c1 = 50;   % F
r2 = 0.003;% ohm
c2 = 40;   %F

rcModelSim = 'rcModelSimulink';
open_system("rcModelSimulink")
% sim(rcModelSim,'FixedStep', '0.1');


% Set parameters
% r0Path = [rcModelSim, '/RC pair/r0'];

% paramValue = get_param('oneRCModel/RC Pair/r0','Resistance');
% set_param(path, "Resistance", r0);


