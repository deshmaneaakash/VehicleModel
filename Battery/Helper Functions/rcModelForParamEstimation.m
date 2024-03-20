function [ccv, vP]  = rcModelForParamEstimation(ocv, current, r0, r1, c1, r2, c2, dt, vP)

    V0  = r0 * current;
    vP1 = RCP(current, r1, c1, vP(1), dt);
    vP2 = RCP(current, r2, c2, vP(2), dt);
    
    vP(1) = vP1;
    vP(2) = vP2;
    ccv = ocv + V0 + vP1 + vP2;

end

function v = RCP(current, r, c, vP, dt)
    
    % Continuous Model
    % rcExp = exp(-dt / (r * c));
    % v = ((current * r) * (1 - rcExp) + (vP * rcExp));

    % Discrete Model
    dV = dt*(-vP / (r * c) + current / c);
    v = vP + dV;
end