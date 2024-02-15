clear
clc

r0 = 4;
rcParams = [6.2  6.2;
            35 35];
ocv = 3;

time = linspace(0, 100, 100);
currentProfile_t = [0 25 26 50 51 100];
currentProfile_A = [0 0 1 1 0 0] * 0.5;
numPairs = 2;
vP = zeros(1, numPairs);

for i = 1:length(time)
    t = time(i);
    current = interp1(currentProfile_t, currentProfile_A, t, "linear");
    
    % Iterating through pairs
    ccv(i) = ocv;

    for rcPair = 1:numPairs
        r = rcParams(1, rcPair);
        c = rcParams(2, rcPair);
        vPolar = RCP(current, r, c, vP(rcPair), t);
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
% ylim([2 20])

function v = RCP(current, r, c, vP, t)
    rcExp = exp(-t / (r * c));
    v = ((current * r) * (1 - rcExp) + (vP * rcExp));
end