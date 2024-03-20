%% EXTENDED KALMAN FILTER

clear
clc
close all

addpath 'C:\Users\deshm\OneDrive\Documents\GitHub\VehicleModel\Battery'
load('C:\Users\deshm\OneDrive\Documents\GitHub\VehicleModel\Data\Panasonic-18650PF-Data-master\Panasonic 18650PF Data\25degC\5 pulse disch\03-11-17_08.47 25degC_5Pulse_HPPC_Pan18650PF.mat')
% % load('"C:\Users\deshm\OneDrive\Documents\GitHub\VehicleModel\Data\Panasonic-18650PF-Data-master\Panasonic 18650PF Data\25degC\5 pulse disch\03-11-17_10.10 3390_dis5_10p.mat"')

%% Preprocessing
cell = batteryCell_PANA18650PF;
time = meas.Time;
[time, uniqueIndices, ~] = unique(time);
voltage = meas.Voltage;
voltage = voltage(uniqueIndices);
currentData = -meas.Current;
currentData = currentData(uniqueIndices);

r0 = 0.005; % ohm
r1 = 0.01;  % ohm
c1 = 8;     % F
r2 = 0.03;  % ohm
c2 = 12;    % F
dt = 0.1;   % s
p = polyfit(cell.soc, cell.ocv.discharge, 6);
vOCV = polyval(p, cell.soc);
dvOCVBySoc = polyder(p);

% A = [1             0                            0;
%      0     exp(-dt / (r1 * c1))                 0;
%      0             0               exp(-dt / (r2 * c2))];

A = [1             0                            0;
     0     (1 - dt / (r1 * c1))                 0;
     0             0               (1 - dt / (r2 * c2))];
% 
% B = [     dt / cell.maxCapacity;
%       r1 * (1 - exp(-dt / (r1 * c1)))
%       r2 * (1 - exp(-dt / (r2 * c2)))];

B = [     dt / cell.maxCapacity;
          dt / c1;
          dt / c2];

D = -r0;

R = 8.4e-4;
Q = diag([1000 0.1 0.1]) * R;

processNoise = 0.1;
sensorNoise = 0.5;

% Initialization 
xUpdated = [100;
           0;
           0];
pUpdated = diag([10 0.1 0.1]);


% Simulation

for i = 1:length(time)
    
    % Inputs
    current = currentData(i);
    u = current;

    % Measurement
    voltageMeasured = voltage(i);
    
    % State Prediction
    xPrediction = A * xUpdated + B * u;
    pPrediction = A * pUpdated * A' + Q;
    C = [polyval(dvOCVBySoc, xUpdated(1)) -1 -1];

    % System output estimate
    y = C * xPrediction + D * u;
    voltagePredicted = y;

    % Kalman Gain
    kalmanGain = pPrediction * C' * (C * pPrediction * C' + R)^-1; 

    % State Update
    I = eye(length(xUpdated));
    xUpdated = xPrediction + kalmanGain * (voltageMeasured - voltagePredicted);
    pUpdated = (I - kalmanGain * C) * pPrediction;

    socEstimate(i) = xUpdated(1);
    stateEstimates(:,i) = xUpdated;
end

figure
plot(time, socEstimate)
