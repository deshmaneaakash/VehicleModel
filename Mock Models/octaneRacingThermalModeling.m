%% THERMAL MODELING OF BATTERY PACK 
% TEAM OCTANE RACING ELECTRIC
% AAKASH DESHMANE
% 
% Use this as a template for your thermal modeling. I have added comments
% where you will have to insert data.

clc
clear
close all

currentData = % insert n by 2 excel data where first column is time in seconds 
              % and second column in current in amps

timeProfile = currentData(:,1);
currentProfile = currentData(:,2);

% Lets assume we are analysing a single module for thermal modeling

% Module Data
numberOfCells = % insert no. of cells in a module
massOfSingleCell = % insert mass of one cell
massOfCells = massOfSingleCell * numberOfCells;

massOfBusbar = % insert mass of busbar for one module. Get this from catia.

resistanceOfCell = % now this is interesting because ideally this changes 
                    % with soc and temperature of cell. But keep this
                    % constant as of now but be aware that you can improve
                    % your modeling by varying this with soc and temp

resistanceOfBusbar = % calculate this by resistivity of busbar and length 
                     % and area of cross section for one module

cPCell = % insert cp of busbar
cPBusbar = % insert cp of busbar
cPAir = % insert cP of air

massFlowRateOfAirflow = % insert mass flow rate, convert cfm to kg/s
UOfAirflow = % use your calculation of U from nusselt number (basic fluid 
            % mechanics). Important: this has to be a function of massFlowRateOfAirflow 

areaOfHeatTransfer = % take this from catia or wherever you have it in your calculations

thermalResistance_busbarToAirflow = 1 / (UOfAirflow * areaOfHeatTransfer);
thermalResistance_cellToBusbar = % this is explained in the document i have sent. A good sanity check is that it 
                                 % should be lower than thermalResistance_busbarToAirflow.

tEquilibrium = 30; % deg C (this is just an assumption)
ambientTemp = 30; % deg C (assumption)

% Simulation 

for timeIndex = 1:length(timeProfile)

    time = timeProfile(timeIndex);
    current = currentProfile(timeIndex);

    % Timestep
    if timeIndex == 1
        timeStep = 1; % this is just initial timestep for code robustness.
                      % change this to whatever value you find common in
                      % data set
    else
        timeStep = time - timeProfile(timeIndex - 1);
    end
    
    % __________________________________________________________________________________________________________________

    % CELL
    
    % Heat Generated
    heatGeneratedInCell = current^2 * resistanceOfCell * timeStep; % Keeping all 
                          % heat units in Joules and not Watts. Whatever
                          % you are comfortable with. Doesnt matter

    % Change in temperature of cell because of heat generated
    changeInTempofCell = heatGeneratedInCell / (massOfCells * cPCell * timeStep); % Q = mCpDt
    cellTemp(timeIndex) = cellTemp(timeIndex-1) + changeInTempofCell;
    
    % Drop in cell temp because some heat is lost to busbar too
    heatLostToBusbar = (cellTemp(timeIndex) - busbarTemp(timeIndex-1)) * timeStep / thermalResistance_cellToBusbar;
    % Q = (dT/R)*dt (in joules)
    % Now this can be negative at times if busbar temp is higher than cell
    % temp. In that case heat transfer is from busbar to cell, that is cell
    % heated by the busbar. This is totally possible

    % If heatLostToBusbar is negative then it is essentially heat gained
    % from busbar. If it is positive then cell temperature drops because of
    % heat lost to busbar
    changeInTempofCell = heatLostToBusbar / (massOfCells * cPCell);

    % Update correct cell temp
    cellTemp(timeIndex) = cellTemp(timeIndex) - changeInTempofCell;

    % __________________________________________________________________________________________________________________

    % BUSBAR

    % Heat input in the busbar 
    % Heat input = (Heat generated from i^2r losses of busbar) + (Heat
    % gained from cell)
    
    heatGainedFromCell = heatLostToBusbar;
    heatGeneratedInBusbar = current^2 * resistanceOfBusbar * timeStep;
    totalHeatInput = heatGeneratedInBusbar + heatGainedFromCell;

    % Change in temperature of busbar because of heat input
    changeInTempOfBusbar = totalHeatInput / (massOfBusbar * cPBusbar); % Q = mCpDt
    busbarTemp(timeIndex) = busbarTemp(timeIndex-1) + changeInTempOfBusbar;
    
    % Some heat is lost from the busbar to airflow
    heatLostToAirflow = (busbarTemp(timeIndex) - ambientTemp) * timeStep / thermalResistance_busbarToAirflow;
    % Assume ambient heat sink to be at a constant temperature

    % Updated busbar temp
    changeInTempOfBusbar = heatLostToAirflow / (massOfBusbar * cPBusbar); % Q = mCpDt
    busbarTemp(timeIndex) = busbarTemp(timeIndex) + changeInTempOfBusbar;

end

% Plot all results

figure
plot(timeProfile, busbarTemp, "LineWidth", 2, "DisplayName", "Busbar Temperature")
plot(timeProfile, cellTemp, "LineWidth", 2, "DisplayName", "Module Temperature")
xlabel("Time [sec]")
ylabel("Temperature [C]")
title("Temperatures during endurance")













