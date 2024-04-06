% Thermal Model of a cell in a pack

clear
close all
clc

%% Initialization of parameters

% Initial Conditions

time = 1:1:10e4;
numCells = 20;
Tequi = 27; % deg C
t1 = Tequi;
t2 = Tequi;
TblockInitial = Tequi;
TroomInitial = Tequi;
TambientInitial = Tequi;

inputs.current = 4;
inputs.mDot = 0.005;

params.cellCp = 900;
params.cellMass = 1;
params.cPFluid = 4200;
params.Rf = 12;
params.Rp = 2;
params.r = 10;
params.Ra = 1;
QOut = 1000; % W

fluidTemps = zeros(numCells + 1, length(time));
cellTemps = zeros(numCells, length(time));
packTemps = zeros(1, length(time));

massOfPack = params.cellMass * numCells;
cPOfPack = 2000;

for t = 1:length(time)
    if t == 1
        fluidTemps(:, t) = Tequi;
        cellTemps(:, t) = Tequi;
        packTemps(t) = Tequi;
        states.Tp = Tequi;

    else 
        params.dt = time(t) - time(t-1);

        % Single Cell Simulation
        
        % States Definition
        totalQFromCells = 0;

        for cellID = 1:numCells

            states.TcPrevious = cellTemps(cellID, t-1);
            if cellID == 1
                states.TfIn = fluidTemps(end, t-1);
            else
                states.TfIn = fluidTemps(cellID - 1, t-1);
            end
            
            states = singleCellThermalModel(states, params, inputs);
    
            totalQFromCells = totalQFromCells + states.QlostToBattery;

            % Store Data
            fluidTemps(cellID, t) = states.TfOut;
            cellTemps(cellID, t) = states.Tc;
        end
        
        states.Tp = (1 / (params.dt + massOfPack * cPOfPack * params.Ra)) * ...
                    (totalQFromCells * params.Ra * params.dt + massOfPack * cPOfPack * params.Ra * packTemps(t-1) ...
                    + TambientInitial * params.dt);
        packTemps(t) = states.Tp;

        % Q out of system acting as a chiller

        dTCoolantAcrossChiller = QOut * params.dt / (params.mDot * params.cPFluid);
        

    end

end

%%

figure
hold on
for cellNum = 1:numCells
    plot(time/3600, cellTemps(cellNum,:), "Linewidth", 2)
end
xlabel("Time [hrs]")
ylabel("Temperature [C]")
title("Cell Temperatures")


%% Local Function

function states = singleCellThermalModel(states, params, inputs)

    states.Tc = getCellTemp(states, params, inputs);
    cellTemp = states.Tc;

    % Q lost to coolant

    QlostToFluid = (cellTemp - states.TfIn) / params.Rf;
    dTFluid = QlostToFluid / (inputs.mDot * params.cPFluid);
    states.TfOut = states.TfIn + dTFluid;

    % Q lost to battery pack

    states.QlostToBattery = (cellTemp - states.Tp) / params.Rp;

end

function temp = getCellTemp(state, param, input)
    term1 = input.current ^ 2 * param.r * param.Rf * param.Rp * param.dt;
    term2 = param.dt * (state.TfIn * param.Rp + state.Tp * param.Rf);
    term3 = param.cellMass * param.cellCp * param.Rp * param.Rf * state.TcPrevious;
    term4 = param.cellMass * param.cellCp * param.Rp * param.Rf + param.Rp * param.dt + param.Rf * param.dt;
    temp = (1 / term4) * (term1 + term2 + term3);
end

