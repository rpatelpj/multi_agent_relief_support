%% Multi Agent Relief Support
restoredefaultpath; close all; clear; clc;
run('setup.m'); % COMMENT WHEN SUBMITTING TO ROBOTARIUM

%% Choose Parameters

% Agent
numAgent = 1; % Number of Agents
A = ones(1, numAgent)'*ones(1, numAgent) - eye(numAgent); % Adjacency Matrix of Graph (info to x info from)
agentMetricVisibleApothem = 1; % Metric Apothem of Visible Map of Agent
gridSensor = true;

% Simulation
contourRes = 5; % Contour Resolution
iteration = 2000; % Total Number of Iterations

% Map
numSink = 1; % Number of Sinks
sinkMetricLen = 0.8; % Metric Length of Square Sink
sinkIdxLen = 11; % Index Length of Square Sink
sinkDepth = 1.5; % Depth of Sink

%% Generate Parameters

fieldDim = ARobotarium.boundaries; % Field Dimensions (xMapMetricMin, xMapMetricMax, yMapMetricMin, yMapMetricMax)
[map, xMapMetricGrid, yMapMetricGrid, metricToIdx] = generateMap(fieldDim(1), fieldDim(2), fieldDim(3), fieldDim(4), numSink, sinkMetricLen, sinkIdxLen, sinkDepth); % Map (y, x)
[agentMetricPos0, agentState] = generateInitialConditions(numAgent, fieldDim(1), fieldDim(2), fieldDim(3), fieldDim(4)); % Agents' Initial Conditions
agentIdxVisibleApothem = floor(agentMetricVisibleApothem.*metricToIdx) + 1; % Index Apothem of Visible Map of Agent

%% Run Driver with Robotarium

roboDrv = Robotarium('NumberOfRobots', numAgent, 'InitialConditions', agentMetricPos0);
agentMetricPosu = roboDrv.get_poses();
roboDrv.step(); % Set agents' initial pose
siToUni = create_si_to_uni_dynamics();
uniClamp = create_uni_barrier_certificate_with_boundary();


contour(xMapMetricGrid, yMapMetricGrid, map, contourRes); % Plot contour of map
agentMetricPosi = agentMetricPosu(1:2, :); % Get agents' single integrator position
handle = scatter(agentMetricPosi(1, :), agentMetricPosi(2, :), 'filled');

currentunits = get(gca,'Units');
set(gca, 'Units', 'Points');
axpos = get(gca,'Position');
set(gca, 'Units', currentunits);
markerWidth = agentMetricVisibleApothem/diff(xlim)*axpos(3); % Calculate Marker width in points

handle.SizeData = markerWidth^2;
handle.MarkerFaceColor = [0.12,0.49,0.65];
handle.MarkerFaceAlpha = 0.2;

for k = 1:iteration
    agentMetricPosu = roboDrv.get_poses();
    agentMetricPosi = agentMetricPosu(1:2, :); % Get agents' single integrator position
    agentMetricVeli = zeros(2, numAgent); % Initialize agents' single integrator velocities
    
    set(handle,'XData',agentMetricPosi(1, :),'YData',agentMetricPosi(2, :))

    for agentN = 1:numAgent
        agentNMetricPosi = agentMetricPosi(1:2, agentN);
        agentNMetricPos0i = agentMetricPos0(1:2, agentN);
        agentAdjacentN = find(A(:, agentN) == 1); % Get set of agents adjacent to agent N
        visibleMapN = readSensorSim(agentNMetricPosi, agentIdxVisibleApothem, map, fieldDim(1), fieldDim(3), metricToIdx, gridSensor); % Extract visible disk of agent
        [agentNState, agentMetricVelNi] = searchRescueController(agentN, agentAdjacentN, visibleMapN, agentNMetricPos0i, agentState, agentMetricPosi, fieldDim(3), fieldDim(4), agentMetricVisibleApothem, sinkMetricLen); % Execute controller
        agentState(agentN) = agentNState;
        agentMetricVeli(:, agentN) = agentMetricVelNi;
    end

    agentMetricVelu = siToUni(agentMetricVeli, agentMetricPosu); % Convert single integrator to unicycle
    agentMetricVelu = uniClamp(agentMetricVelu, agentMetricPosu); % Impose inter-agent barrier and field boundary
    roboDrv.set_velocities(1:numAgent, agentMetricVelu);
    roboDrv.step();
    drawnow;
end

roboDrv.debug(); % Debug simulation; COMMENT WHEN SUBMITTING TO ROBOTARIUM