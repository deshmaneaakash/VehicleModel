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
currentData = meas.Current;
currentData = currentData(uniqueIndices);

%% System

r0 = 0.005; % ohm
r1 = 0.01;  % ohm
c1 = 8;     % F
r2 = 0.03;  % ohm
c2 = 12;    % F
dt = 0.1;   % s
vOCVPolynomial = polyfit(cell.soc, cell.ocv.discharge, 6);
vOCV = polyval(vOCVPolynomial, cell.soc);

A = [1             0                            0;
     0     (1 - dt / (r1 * c1))                 0;
     0             0               (1 - dt / (r2 * c2))];

B = [ dt / (cell.maxCapacity * 3600);
      dt / c1;
      dt / c2];

D = -r0;

R = 0.1;
Q = diag([1e-5 1e-3 1e-3]);

% Initialization 
initialSoc = 100;
xUpdated = [initialSoc;
           0;
           0];
pUpdated = diag([10 0.1 0.1]);
uncontrollableStates = length(A) - rank(ctrb(A,B));


%% Simulation

for i = 1:length(time)
    
    % Inputs
    current = currentData(i);
    u = current;

    % Measurement
    voltageMeasured = voltage(i);
    
    % State Prediction
    xPrediction = A * xUpdated + B * u;
    pPrediction = A * pUpdated * A' + Q;

    % System output estimate
    socPredicted = xUpdated(1);
    dvOCVBySoc = polyval(vOCVPolynomial, socPredicted) / socPredicted;
    C = [dvOCVBySoc -1 -1];
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
    kalmanGains(i) = kalmanGain(1);
    variances(i) = pUpdated(1);
    unobservableStates(i) = length(A) - rank(obsv(A,C));

    % Coulomb Counting

    if i == 1
        CCSoc(i) = initialSoc;
    else
        dt = time(i) - time(i-1);
        CCSoc(i) = CCSoc(i-1) + dt * u * 100 / (cell.maxCapacity * 3600);
    end

end


figure
hold on
plot(time / 3600, socEstimate, 'LineWidth', 2, "DisplayName", "SoC estimate from EKF")
plot(time / 3600, CCSoc, 'LineWidth', 2, "DisplayName", "SoC estimate from Coulomb Counting")
xlabel("Time [Hrs]")
ylabel("SOC [%]")
title("SOC Estimates")

% figure
% plot(time / 3600, kalmanGains, 'LineWidth', 2)
% xlabel("Time [Hrs]")
% ylabel("Kalman Gain")
% title("Kalman Gain vs Time")
% 
% figure
% plot(time / 3600, variances, 'LineWidth', 2)
% xlabel("Time [Hrs]")
% ylabel("Variance")
% title("Variance of SOC vs Time")


