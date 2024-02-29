clear
clc

load('C:\Users\deshm\OneDrive\Documents\GitHub\VehicleModel\EVsim\Panasonic-18650PF-Data-master\Panasonic 18650PF Data\25degC\5 pulse disch\03-11-17_08.47 25degC_5Pulse_HPPC_Pan18650PF.mat')

%%
time = meas.Time;
[time, uniqueIndices, ~] = unique(time);
simulationTime = max(time);

voltage = meas.Voltage;
voltage = voltage(uniqueIndices);

current = meas.Current;
current = current(uniqueIndices);

inputData = [time, current];
cell = batteryCell_PANA18650PF;
soc = cell.soc;
ocv = cell.ocv.discharge;
initialSoc = 100;

% Initial parameters for the model
r0 = 0.03; % ohm
r1 = 0.05; % ohm
c1 = 15;   % F
r2 = 0.05;% ohm
c2 = 14;   %F

out = sim("rcModelSimulink", simulationTime);

%%

figure
hold on
plot(out.ccv.Time, out.ccv.Data, "LineWidth", 2, "DisplayName", "Model")
plot(time, voltage, "LineWidth", 2, "DisplayName", "Test")
legend

