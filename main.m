%% Multi Agent Relief Support
% restoredefaultpath; close all; clear; clc; % COMMENT WHEN SUBMITTING TO ROBOTARIUM
% run('setup.m'); % COMMENT WHEN SUBMITTING TO ROBOTARIUM

%% Choose Parameters

% Agent
numAgent = 10; % Number of Agents
A = ones(1, numAgent)'*ones(1, numAgent) - eye(numAgent); % Adjacency Matrix of Graph (info to x info from), Complete Graph
% A = diag(ones(1, (numAgent - 1)), 1) + diag(ones(1, (numAgent - 1)), -1); % Adjacency Matrix of Graph (info to x info from), Cycle Graph
% A(numAgent, 1) = 1;
% A(1, numAgent) = 1;
agentMetricVisibilityApothem = 0.1; % Metric Apothem of Visible Map of Agent
gridSensorRange = false; % If true, use grid sensor range. If false, use circular sensor range.

% Simulation
contourRes = 5; % Contour Resolution
iteration = 3000; % Total Number of Iterations

% Sink
numSink = 10; % Number of Sinks
sinkMetricLen = 0.3; % Metric Length of Square Sink

% Sink, Rarely Changed
sinkIdxLen = 11; % Index Length of Square Sink
sinkDepth = 1.5; % Depth of Sink

%% Robotarium Parameters

fieldDim = [-1.6, 1.6, -1, 1]; % Field Dimensions (xMapMetricMin, xMapMetricMax, yMapMetricMin, yMapMetricMax), ARobotarium.boundaries
agentSize = 0.11; % Size of agent, ARobotarium.robot_diameter
sinkMetricLen = max(sinkMetricLen, agentSize);

%% Generate Parameters

[map, xMapMetricGrid, yMapMetricGrid, metricToIdx] = generateMap(fieldDim(1), fieldDim(2), fieldDim(3), fieldDim(4), numSink, sinkMetricLen, sinkIdxLen, sinkDepth); % Map (y, x)
[agentMetricPos0, agentState] = generateInitialConditions(numAgent, fieldDim(1), fieldDim(2), fieldDim(3), fieldDim(4)); % Agents' Initial Conditions
agentIdxVisibilityApothem = floor(agentMetricVisibilityApothem.*metricToIdx) + 1; % Index Apothem of Visible Map of Agent

%% Run Driver with Robotarium

% Initialize Robotarium
roboDrv = Robotarium('NumberOfRobots', numAgent, 'InitialConditions', agentMetricPos0);
agentMetricPosu = roboDrv.get_poses();
roboDrv.step(); % Set agents' initial pose
siToUni = create_si_to_uni_dynamics();
uniClamp = create_uni_barrier_certificate_with_boundary();

% Plot contour of map
contour(xMapMetricGrid, yMapMetricGrid, map, contourRes);

% Initialize plot for visible map of agents
agentMetricPosi = agentMetricPosu(1:2, :);
handle = scatter(agentMetricPosi(1, :), agentMetricPosi(2, :), 'filled');
currentunits = get(gca,'Units');
set(gca, 'Units', 'Points');
axpos = get(gca,'Position');
set(gca, 'Units', currentunits);
markerWidth = axpos(3).*(2.*agentMetricVisibilityApothem)/diff(xlim); % Calculate marker width in points
handle.SizeData = markerWidth.^2;
handle.MarkerFaceColor = [0.12, 0.49, 0.65];
handle.MarkerFaceAlpha = 0.5;

for k = 1:iteration
    agentMetricPosu = roboDrv.get_poses();
    agentMetricPosi = agentMetricPosu(1:2, :); % Get agents' single integrator position
    agentMetricVeli = zeros(2, numAgent); % Initialize agents' single integrator velocities

    set(handle, 'XData', agentMetricPosi(1, :), 'YData', agentMetricPosi(2, :)); % Load agent positions for visible map plots

    for agentNIdx = 1:numAgent
        agentNMetricPosi = agentMetricPosi(1:2, agentNIdx);
        agentAdjacentNIdx = find(A(:, agentNIdx) == 1); % Get set of agents adjacent to agent N
        visibleMapN = readSensorSim(agentNMetricPosi, agentIdxVisibilityApothem, map, fieldDim(1), fieldDim(3), metricToIdx, gridSensorRange); % Extract visible disk of agent
        [agentNState, agentNMetricVeli] = searchRescueController(agentNIdx, agentAdjacentNIdx, visibleMapN, agentState, agentMetricPosi, fieldDim(1), fieldDim(2), fieldDim(3), fieldDim(4), metricToIdx, agentMetricVisibilityApothem, sinkMetricLen); % Execute controller
        agentState(agentNIdx) = agentNState;
        agentMetricVeli(:, agentNIdx) = agentNMetricVeli;
    end

    agentMetricVelu = siToUni(agentMetricVeli, agentMetricPosu); % Convert single integrator to unicycle
    agentMetricVelu = uniClamp(agentMetricVelu, agentMetricPosu); % Impose inter-agent barrier and field boundary
    roboDrv.set_velocities(1:numAgent, agentMetricVelu);
    roboDrv.step();
    drawnow; % Update visible map of agents
end

% roboDrv.debug(); % Debug simulation; COMMENT WHEN SUBMITTING TO ROBOTARIUM