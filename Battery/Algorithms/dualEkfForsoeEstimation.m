%% EXTENDED KALMAN FILTER FOR SOE ESTIMATION

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

r0 = 0.001;
r1 = 0.0005;
c1 = 0.3;
r2 = 0.05;  
c2 = 4;
dt = 0.1;   % s
efficiency = 0.99;
vOCVPolynomial = polyfit(cell.soc, cell.ocv.discharge, 6);
vOCV = polyval(vOCVPolynomial, cell.soc);
maxEnergy = cell.ratedCapacity * cell.maxV;

A = [1             0               0                    0;
     0             1               0                    0;           
     0             0     (1 - dt / (r1 * c1))           0;
     0             0               0           (1 - dt / (r2 * c2))];

B = [ 0                          dt*efficiency/(cell.maxCapacity * 3600);
      dt*100/(maxEnergy*3600)                               0;
      0                                             dt/c1;
      0                                            dt/c2];

D = [0 -r0];

R = 0.1;
Q = diag([1e-5 1e-5 1e-3 1e-3]);

% Initialization 
initialSoc = 100;
initialEnergy = 100;
xUpdated = [initialSoc;
            initialEnergy;
            0;
            0];
pUpdated = diag([10 10 0.1 0.1]);
uncontrollableStates = length(A) - rank(ctrb(A,B));


%% Simulation

for i = 1:length(time)
    
    % Inputs
    current = currentData(i);

    % Measurement
    voltageMeasured = voltage(i);
    u = [voltageMeasured * current 
            current];
    
    % State Prediction
    xPrediction = A * xUpdated + B * u;
    pPrediction = A * pUpdated * A' + Q;

    % System output estimate
    socPredicted = xUpdated(1);
    soePredicted = xUpdated(2);

    % Calculating Jacobian
    if i == 1
        dvOCVBySoc = polyval(vOCVPolynomial, socPredicted) / socPredicted;
        dSocBySoe = 1;
    else
        % Previous SoC
        prevSoc = socEstimate(i-1);
        dSoc = socPredicted - prevSoc;

        % Previous SoE
        prevSoe = soeEstimate(i-1);
        dSoe = soePredicted - prevSoe;

        if dSoc == 0
            dvOCVBySoc = polyval(vOCVPolynomial, socPredicted) / socPredicted;
            dSocBySoe = 1;
        else
            dvOcv = polyval(vOCVPolynomial, socPredicted) - polyval(vOCVPolynomial, prevSoc);
            dvOCVBySoc = dvOcv / dSoc;
            dSocBySoe = dSoc / dSoe;
        end
    end
    dvOCVBySoe = dvOCVBySoc * dSocBySoe;
    C = [dvOCVBySoc 0 -1 -1]; % Jacobian of C matrix

    % Output Estimate
    y = C * xPrediction + D * u;
    voltagePredicted = y;

    % Kalman Gain
    kalmanGain = pPrediction * C' * (C * pPrediction * C' + R)^-1; 

    % State Update
    I = eye(length(xUpdated));
    xUpdated = xPrediction + kalmanGain * (voltageMeasured - voltagePredicted);
    pUpdated = (I - kalmanGain * C) * pPrediction;

    socEstimate(i) = xUpdated(1);
    soeEstimate(i) = xUpdated(2);
    stateEstimates(:,i) = xUpdated;
    kalmanGains(i) = kalmanGain(1);
    variances(i) = pUpdated(1);
    % unobservableStates(i) = length(A) - rank(obsv(A,C));

    % Coulomb Counting

    if i == 1
        CCSoc(i) = initialSoc;
    else
        dt = time(i) - time(i-1);
        CCSoc(i) = CCSoc(i-1) + dt * u(2) * 100 / (cell.maxCapacity * 3600);
    end

end

Ah = meas.Ah(uniqueIndices);
Wh = meas.Wh(uniqueIndices);
soeFromWh = 100 + Wh * 100 / (cell.ratedCapacity * cell.maxV);
socFromAh = 100 + Ah * 100 / cell.ratedCapacity;

figure
hold on
plot(time / 3600, socEstimate, 'LineWidth', 2, "DisplayName", "SoC estimate from EKF")
plot(time / 3600, CCSoc, 'LineWidth', 2, "DisplayName", "SoC estimate from Coulomb Counting")
plot(time / 3600, socFromAh, 'LineWidth', 2, "DisplayName", "SoC from Ah reading")
xlabel("Time [Hrs]")
ylabel("SOC [%]")
title("SOC Estimates")
legend

figure
hold on
plot(time / 3600, soeEstimate, 'LineWidth', 2, "DisplayName", "SoE estimate from EKF")
plot(time / 3600, soeFromWh, 'LineWidth', 2, "DisplayName", "SoE from Wh reading")
xlabel("Time [Hrs]")
ylabel("SOE [%]")
title("SOE Estimates")
legend

% figure
% hold on
% plot(meas.Time, meas.Voltage, "DisplayName", "Voltage")
% yyaxis right
% plot(meas.Time, meas.Current, "DisplayName", "Current")
% legend

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


