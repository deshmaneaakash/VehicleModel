clear
clc

load('C:\Users\deshm\OneDrive\Documents\GitHub\VehicleModel\EVsim\Panasonic-18650PF-Data-master\Panasonic 18650PF Data\25degC\C20 OCV and 1C discharge tests_start_of_tests\05-08-17_13.26 C20 OCV Test_C20_25dC.mat')

timeData = meas.Time;
currentData = meas.Current;
voltageData = meas.Voltage;
AhData = meas.Ah;
initialSoc = 100;
cellCapacity = 2.9677 * 3600;

for t = 1:length(timeData)
    
    % Coulomb Counting
    current = currentData(t);
    Ah = AhData(t);
    if t == 1
        soc(t) = initialSoc;
    else
        dt = timeData(t) - timeData(t-1);
        soc(t) = soc(t-1) + dt * current * 100 / cellCapacity;
    end

end

dischargeCurrentIndices = find(currentData<0);
chargeCurrentIndices = find(currentData>0);

dischargeSoc = soc(dischargeCurrentIndices);
chargeSoc = soc(chargeCurrentIndices);

dischargeOCV = voltageData(dischargeCurrentIndices);
chargeOCV = voltageData(chargeCurrentIndices);

dischargeNegativeSocIndices = find(dischargeSoc < 0);
dischargeSoc(dischargeNegativeSocIndices) = [];
dischargeOCV(dischargeNegativeSocIndices) = [];

chargeNegativeSocIndices = find(chargeSoc < 0);
chargeSoc(chargeNegativeSocIndices) = [];
chargeOCV(chargeNegativeSocIndices) = [];

chargeOCV = interp1(chargeSoc, chargeOCV, dischargeSoc, "linear", max(chargeOCV));
chargeOCV(end) = chargeOCV(end - 1);
commonSoc = dischargeSoc;

%% Plotting

figure
hold on
plot(commonSoc, dischargeOCV, "LineWidth", 2, 'Color', 'r', 'DisplayName', "Discharge OCV")
plot(commonSoc, chargeOCV, "LineWidth", 2, "Color", 'b', 'DisplayName', "Charge OCV")
xlabel('SOC')
ylabel('Voltage')
title("OCV curves")
legend



