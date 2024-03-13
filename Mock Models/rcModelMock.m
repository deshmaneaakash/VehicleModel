clear
clc

r0 = 0.01;
rcParams = [0.03 0.04;
            60 60];
ocv = 3;

load('C:\Users\deshm\OneDrive\Documents\GitHub\VehicleModel\Data\Panasonic-18650PF-Data-master\Panasonic 18650PF Data\25degC\5 pulse disch\03-11-17_08.47 25degC_5Pulse_HPPC_Pan18650PF.mat')

% time = meas.Time;
% voltage = meas.Voltage;
% currentData = meas.Current;

time = linspace(0, 100, 100);
currentProfile_t = [0 25 26 50 51 100];
currentProfile_A = [0 0 1 1 0 0] * 0.5;

% currentProfile_t = time;
% currentProfile_A = currentData;

numPairs = 2;
vP = zeros(1, numPairs);

for i = 1:length(time)
    t = time(i);
    if i == 1
        dt = 0.1;
    else
        dt = time(i) - time(i-1);
    end

    current = interp1(currentProfile_t, currentProfile_A, t, "linear");
    % current = currentData(i);

    % Iterating through pairs
    ccv(i) = ocv;

    for rcPair = 1:numPairs
        r = rcParams(1, rcPair);
        c = rcParams(2, rcPair);
        vPolar = RCP(current, r, c, vP(rcPair), dt);
        ccv(i) = ccv(i) + vPolar;
        vP(rcPair) = vPolar;
    end

    ccv(i) = ccv(i) + current * r0;
end

figure(1)
hold on
grid on
plot(time, ccv, "LineWidth", 2)
xlabel("Time [s]")
ylabel("Voltage [V]")
title("RC Pair Response")
% ylim([-7 3])
% xlim([15545 15560])

function v = RCP(current, r, c, vP, dt)
    % Continous model
    rcExp = exp(-dt / (r * c));
    v = ((current * r) * (1 - rcExp) + (vP * rcExp));

    % Discrete Model
    % dV = dt*(-vP / (r * c) + current / c);
    % v = vP + dV;

end