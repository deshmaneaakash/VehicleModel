function ccv = rcModel(ocv, current, t)

r1 = 10;
r2 = 5;
c1 = 12;
c2 = 10;

% time = linspace(0, 100, 100);


vP = [0 0];
% for i = 1:length(time)
    % t = time(i);
    % current = interp1(currentProfile_t, currentProfile_A, t, "linear");

vP1 = RCP(current, r1, c1, vP(1), t);
vP2 = RCP(current, r2, c2, vP(2), t);

% vP(1) = vP1;
% vP(2) = vP2;
ccv = ocv + vP1 + vP2;

% end

end

function v = RCP(current, r, c, vP, t)
    rcExp = exp(-t / (r * c));
    v = ((current * r) * (1 - rcExp) + (vP * rcExp));
end