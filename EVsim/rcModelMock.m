clear
clc

r1 = 17.44 * 0.001;
r2 = 5;
c1 = 12;
c2 = 10;
ocv = 3;

time = linspace(0, 100, 100);
currentProfile_t = [0 25 26 50 51 100];
currentProfile_A = [0 0 1 1 0 0] * 0.5;

vP = [0 0];
for i = 1:length(time)
    t = time(i);
    current = interp1(currentProfile_t, currentProfile_A, t, "linear");

    vP1 = RCP(current, r1, c1, vP(1), t);
    vP2 = RCP(current, r2, c2, vP(2), t);

    vP(1) = vP1;
    vP(2) = vP2;
    ccv(i) = ocv + vP1 + vP2;

end

figure
grid
plot(time, ccv, "LineWidth", 2)
xlabel("Time [s]")
ylabel("Voltage [V]")
title("RC Pair Response")
ylim([2 20])

function v = RCP(current, r, c, vP, t)
    rcExp = exp(-t / (r * c));
    v = ((current * r) * (1 - rcExp) + (vP * rcExp));
end