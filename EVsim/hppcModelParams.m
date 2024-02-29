clear
clc

load('C:\Users\deshm\OneDrive\Documents\GitHub\VehicleModel\EVsim\Panasonic-18650PF-Data-master\Panasonic 18650PF Data\25degC\5 pulse disch\03-11-17_08.47 25degC_5Pulse_HPPC_Pan18650PF.mat')

%%
time = meas.Time;
voltage = meas.Voltage;
current = meas.Current;
inputData = [time, current];
cell = batteryCell_PANA18650PF;
soc = cell.soc;
ocv = cell.ocv.discharge;
initialSoc = 100;

% Initial parameters for the model
r0 = 0.02; % ohm
r1 = 0.01; % ohm
c1 = 50;   % F
r2 = 0.003;% ohm
c2 = 40;   %F

rcModelSim = 'rcModelSimulink';
% open_system("rcModelSimulink")

%%

figure
hold on
plot(out.ccv.Time, out.ccv.Data, "LineWidth", 2, "DisplayName", "Model")
plot(time, voltage, "LineWidth", 2, "DisplayName", "Test")
legend

