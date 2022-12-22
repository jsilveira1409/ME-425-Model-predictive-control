%
% Trace out an EPFL in ref_time seconds
%
function Ref4 = ref_EPFL(t, roll_max, tilt)

ref_time = 30;

if nargin < 2
    roll_max = deg2rad(15);
end
if nargin < 3
    tilt = true;
end

% Coordinates (x, y, heading)
coords = [ ...
    % E
    1.5 2   0;
    0   2   0;
    0   1   0;
    1.5 1   0;
    0   1   0;
    0   0   0;
    %1.5 0   0;
    % P
    2   0   0.5;
    2   2   0.5;
    3.5 2   0.5;
    3.5 1   0.5;
    2   1   0.5;
    %3.5 1   0.5;
    % F
    4   1   -1;
    4   0   -1;
    4   1   -1;
    5.5 1   -1;
    4   1   -1
    4   2   -1;
    %5.5 2   1;
    % L
    6   2   0;
    6   0   0;
    7.5 0   0;
    ];
coords(:,1:2) = coords(:,1:2) - [1.5 2];
coords(:,1:2) = coords(:,1:2) / 2.5;
coords(:,3) = coords(:,3) * rad2deg(roll_max);

nCoords = size(coords, 1);

% Break the path into legs, compute their end times such that
% the end of the path is reached at ref_time
legs = coords(2:end,1:2) - coords(1:end-1,1:2);
distances = vecnorm(legs, 2, 2);
leg_endtimes = [0; ref_time * cumsum(distances) / sum(distances)]';

% Find target index for each time point
target_id = sum(t(:) > leg_endtimes, 2) + 1;
% Limit target_id to final point
target_id = min(nCoords, target_id);

% Return target coordinates for each time point
XZ = [coords(target_id, 1:2)];
XYZ = [XZ(1), 0, XZ(2)];
Roll = deg2rad(coords(target_id,3));

if tilt
    % Rotate
    alpha = deg2rad(-15);
    beta = deg2rad(19);
    gamma = deg2rad(-24);
    
    T = Rocket.eul2mat([alpha, beta, gamma]);
    XYZ = (T * XYZ')';
end
Ref4 = [XYZ, Roll]; % 4D X 0 Z roll

end