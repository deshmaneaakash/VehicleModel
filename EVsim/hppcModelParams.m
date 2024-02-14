clear
clc

rawData = readtable('LG 18651HG2\LG 18650HG2 Li-ion Battery Data and Example Deep Neural Network xEV SOC Estimator Script\LG_HG2_Original_Dataset_McMasterUniversity_Jan_2020\25degC\549_HPPC.csv');
rawData(1,:) = [];

%%
time = seconds(table2array((rawData(:,"ProgTime"))));
voltage = table2array((rawData(:,"Voltage")));
current = table2array((rawData(:,"Current")));

figure
hold on
plot(time, voltage, 'LineWidth', 2)
plot(time, current, 'LineWidth', 2)