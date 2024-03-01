clear
clc

load('C:\Users\deshm\OneDrive\Documents\GitHub\VehicleModel\EVsim\Panasonic-18650PF-Data-master\Panasonic 18650PF Data\25degC\5 pulse disch\03-11-17_08.47 25degC_5Pulse_HPPC_Pan18650PF.mat')

%%

time = meas.Time;
[time, uniqueIndices, ~] = unique(time);

voltage = meas.Voltage;
voltage = voltage(uniqueIndices);

current = meas.Current;
current = current(uniqueIndices);
[peaks, xLocOfPeaks] = findpeaks(-current, "MinPeakDistance", 100);
findpeaks(-current, "MinPeakDistance", 100);
seperationBuffer = 100;


%%
pulseEndIndices = xLocOfPeaks + seperationBuffer;
pulsePackets = struct;
pulseStartIndex = 1;

for endIndex = 1:length(pulseEndIndices)
    pulseEndIndex = pulseEndIndices(endIndex);
    pulsePackets(endIndex).time = time(pulseStartIndex:pulseEndIndex);
    pulsePackets(endIndex).current = current(pulseStartIndex:pulseEndIndex);
    pulsePackets(endIndex).voltage = voltage(pulseStartIndex:pulseEndIndex);
    pulseStartIndex = pulseEndIndex + 1;
end

figure;
plot(pulsePackets(2).time, pulsePackets(2).current)

inputData = [pulsePackets(1).time, pulsePackets(1).current];
measuredTime = pulsePackets(1).time;
measuredVoltage = pulsePackets(1).voltage;
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

simulationTime = max(pulsePackets(1).time);
% open_system("rcModelSimulink")
% out = sim("rcModelSimulink", simulationTime);

%%

figure
hold on
plot(out.ccv.Time, out.ccv.Data, "LineWidth", 2, "DisplayName", "Model")
plot(time, voltage, "LineWidth", 2, "DisplayName", "Test")
legend

