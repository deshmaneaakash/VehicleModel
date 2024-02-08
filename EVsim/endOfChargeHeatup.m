data = readmatrix('Data.csv');
vSoc = data(3:end, 1);
% vOcv = data(3:end, 2);
vOcv = ones(length(vSoc)) * 3;

time = linspace(1, 100, 100);
initialSoc = 0;
% current  = 0.5;

currentProfile_t = [0 25 26 50 51 100];
currentProfile_A = [0 0 1 1 0 0];
soc = [];
cellCapacity = 100; % Ah

for t = 1:length(time)
    
    % Coulomb Counting
    current = interp1(currentProfile_t, currentProfile_A, t, "linear");
    if t == 1
        soc(t) = initialSoc;
    else
        soc(t) = soc(t-1) + t * current / cellCapacity;
    end

    ocv = interp1(vSoc, vOcv, soc(t), "linear");
    vTerminal(t) = rcModel(ocv, current, t);

end

figure
grid
plot(time, vTerminal, "LineWidth", 2)
xlabel("Time [s]")
ylabel("Terminal Voltage [V]")
title("RC Pair Response")
